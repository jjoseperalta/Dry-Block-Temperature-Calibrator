#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(Settings& settings) : settings(settings) {
    integralLimit = 0.0f;
    reset();
}

void PIDController::reset() {
    integral = 0.0f;
    previous_error = 0.0f;
    previous_pv = 0.0f;
}

float PIDController::calculate(float setpoint, float process_variable, float dt) {
    // Lectura de parámetros (Asumo que esta parte lee desde la clase settings)
    float kp = settings.getPidKp();
    float ti = settings.getPidTi();
    float td = settings.getPidTd();

    // Protección: Asegurar que dt no es cero
    if (dt <= 0.0f) dt = 0.001f; 

    float error = setpoint - process_variable;

    // --- 1. Término Proporcional (P) ---
    float p_out = kp * error;

    // --- 2. Término Derivativo (D) ---
    // Derivativo en la medición (PV) para evitar picos en el cambio de setpoint
    float derivative = (process_variable - previous_pv) / dt;
    float d_out = -kp * td * derivative; // Signo negativo para frenar el calentamiento

    // --- 3. Término Integral (I) con Anti-Windup Clamping ---
    float i_out = 0.0f;
    float tentative_output = p_out + d_out; // Salida P+D antes de incluir la Integral
    
    if (ti > 1e-6f) {
        // Cálculo de la acción I (Ki * integral)
        float kp_over_ti = kp / ti; 
        
        // 1. Determinar si la salida P+D+I estaría saturada
        // Usamos el estado actual de la integral para ver si contribuye a la saturación.
        float tentative_output_full = tentative_output + (kp_over_ti * integral);

        // 2. Lógica de Anti-Windup (Clamping)
        // Solo integramos si:
        // a) La salida NO está saturada (entre -100 y 100), O
        // b) El error es NEGATIVO (necesitamos enfriar) Y la salida está saturada POSITIVAMENTE (>100),
        // c) El error es POSITIVO (necesitamos calentar) Y la salida está saturada NEGATIVAMENTE (<-100).
        // (La lógica b y c permite desenrollar el windup si la saturación es incorrecta)
        
        bool output_clamped = false;
        if (tentative_output_full > 100.0f && error > 0.0f) {
            output_clamped = true;
        } else if (tentative_output_full < -100.0f && error < 0.0f) {
            output_clamped = true;
        }

        if (!output_clamped) {
             // Acumular el error si no estamos saturados
             integral += error * dt;
        }

        // 3. Calcular la salida final del término I
        i_out = kp_over_ti * integral;
    } else {
        // Si Ti es cero o muy pequeño, la integral no se usa y se limpia
        integral = 0.0f; 
    }

    // --- 4. Salida Total ---
    float output = p_out + i_out + d_out;

    // --- 5. Clamping Final (Saturación del Driver) ---
    // Limitar la salida que va al driver al rango [-100, 100]
    if (output > 100.0f) output = 100.0f;
    else if (output < -100.0f) output = -100.0f;

    previous_error = error;
    previous_pv = process_variable;

    return output;
}