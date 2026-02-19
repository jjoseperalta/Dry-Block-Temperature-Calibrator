#include "PIDController.h"
#include "Logger.h"
#include <Arduino.h>
#include <cmath>

PIDController::PIDController(Settings &settings) : settings(settings) {
  reset();
}

void PIDController::reset() {
  integral = 0.0f;
  lastError = 0.0f;
  previous_pv = 0.0f;
  inFineZone = false;
}

PIDGains PIDController::computeGains(float error, bool fineZone) {
  float kp = settings.getPidKp();
  float ti = settings.getPidTi();
  float td = settings.getPidTd();

  bool heating = (error >= 0.0f);

  PIDGains g;

  if (heating) {
    g.kp =
        kp *
        (fineZone ? 0.8f : 1.2f); // 0.8f : 1.5f OK - pero 1.0f : 1.2f más suave
    g.ti =
        ti *
        (fineZone ? 0.2f : 0.8f); // 0.3f : 0.7f OK - pero 0.5f : 1.0f más suave
    g.td =
        td *
        (fineZone ? 1.0f : 1.0f); // 1.5f : 0.0f OK - pero 1.0f : 1.0f más suave
  } else {
    // ENFRIAR: Suave en potencia, pero muy alto en frenado (D)
    g.kp = kp * (fineZone ? 0.5f : 1.2f);
    g.ti = ti * (fineZone ? 1.2f : 1.0f);
    g.td = td * (fineZone ? 2.0f : 1.0f);
  }

  return g;
}

float PIDController::calculate(float setpoint, float process_variable,
                               float dt) {
  if (dt <= 0.0f)
    dt = 0.01f;

  float error = setpoint - process_variable;

  // --- FINE ZONE CON HISTERESIS ---
  inFineZone = (fabs(error) < 1.0f);
  // static bool fineState = false;
  // if (fineState) {
  //   if (fabs(error) > 1.2f)
  //     fineState = false;
  // } else {
  //   if (fabs(error) < 0.8f)
  //     fineState = true;
  // }
  // inFineZone = fineState;

  PIDGains g = computeGains(error, inFineZone);

  // --- PROPORCIONAL ---
  float p_out = g.kp * error;

  // --- DERIVADA ---
  float derivative = (process_variable - previous_pv) / dt;
  derivative = constrain(derivative, -1.0f, 1.0f);
  float d_out = -g.kp * g.td * derivative;

  // --- INTEGRAL CON ANTI-WINDUP ---
  // float i_out = 0.0f;
  if (g.ti > 1e-6f) {
    if (fabs(error) < 3.0f) // fabs(error) < 2.0f OK - pero 3.0f más suave
    {
      integral += error * dt;

      integral = constrain(integral, -5.0f, 100.0f); //-20.0f, 75.0f OK - pero 60.0f más suave
    }
    // i_out = (g.kp / g.ti) * integral;
  } else
    integral = 0.0f;

  float i_out = (g.ti > 1e-6f) ? (g.kp / g.ti) * integral : 0.0f;

  // --- SALIDA ---
  float output = p_out + i_out + d_out;

  // Anti-windup por saturación
  output = constrain(output, -100.0f, 100.0f);
  // if (output > 100.0f) {
  //   output = 100.0f;
  //   if (error > 0)
  //     integral -= error * dt;
  // } else if (output < -100.0f) {
  //   output = -100.0f;
  //   if (error < 0)
  //     integral -= error * dt;
  // }

  // --- NUEVA LÓGICA DE ESTABILIZACIÓN ---
  // static float filteredOutput = 0.0f;
  // float alpha = 0.2f; // Factor de suavizado (0.1 a 0.3)

  // 1. Aplicamos un filtro de paso bajo siempre para evitar el serrucho
  // filteredOutput = (output * alpha) + (filteredOutput * (1.0f - alpha));

  // // 2. Deadband más amplia (0.15°C)
  // if (fabs(error) < 0.15f) {
  //     // Si estamos muy cerca, mantenemos el filtro muy pesado 
  //     // para que la línea en la gráfica sea casi plana
  //     output = filteredOutput;
  // } else {
  //     output = filteredOutput; 
  // }

  previous_pv = process_variable;
  return output;
}
