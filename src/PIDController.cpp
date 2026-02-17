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
    // CALENTAR: Agresivo para llegar, predictivo para frenar
<<<<<<< HEAD
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
=======
    g.kp = kp * (fineZone ? 1.0f : 1.0f);//0.8f : 1.5f OK - pero 1.0f : 1.2f más suave
    g.ti = ti * (fineZone ? 0.5f : 1.0f);//0.3f : 0.7f OK - pero 0.5f : 1.0f más suave
    g.td = td * (fineZone ? 1.0f : 1.0f);//1.5f : 0.0f OK - pero 1.0f : 1.0f más suave
  } else {
    // ENFRIAR: Suave en potencia, pero muy alto en frenado (D)
    g.kp = kp * (fineZone ? 0.4f : 1.2f);
    g.ti = ti * (fineZone ? 1.5f : 0.8f);
>>>>>>> 709c66cf67a90b0d24b2924397561c153b988078
    g.td = td * (fineZone ? 2.0f : 1.0f);
  }

  return g;
}

float PIDController::calculate(float setpoint, float process_variable,
                               float dt) {
  if (dt <= 0.0f)
<<<<<<< HEAD
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

=======
    dt = 0.01f; // Un dt mínimo más realista que 0.001

  float error = setpoint - process_variable;

  // ACTUALIZAR ESTO: Detectar si estamos a menos de 1 grado
  // Reducir el radio de la zona fina para que empuje fuerte hasta el final
  inFineZone = (fabs(error) < 1.0f); // Antes era 1.0f

  // --- 1. GANANCIAS ADAPTATIVAS ---
  // IMPORTANTE: Aquí es donde podemos decirle que si estamos en HOLD, use
  // ganancias más suaves
>>>>>>> 709c66cf67a90b0d24b2924397561c153b988078
  PIDGains g = computeGains(error, inFineZone);

  // --- PROPORCIONAL ---
  float p_out = g.kp * error;

  // --- DERIVADA ---
  float derivative = (process_variable - previous_pv) / dt;
<<<<<<< HEAD
  derivative = constrain(derivative, -1.0f, 1.0f);
=======

  // Limitemos la derivada para evitar ruidos de fase (el famoso -100.00)
  derivative = constrain(derivative, -0.5f, 0.5f);

>>>>>>> 709c66cf67a90b0d24b2924397561c153b988078
  float d_out = -g.kp * g.td * derivative;

  // --- INTEGRAL CON ANTI-WINDUP ---
  // float i_out = 0.0f;
  if (g.ti > 1e-6f) {
<<<<<<< HEAD
    if (fabs(error) < 3.0f) // fabs(error) < 2.0f OK - pero 3.0f más suave
    {
      integral += error * dt;

      integral = constrain(integral, -5.0f, 100.0f); //-20.0f, 75.0f OK - pero 60.0f más suave
    }
    // i_out = (g.kp / g.ti) * integral;
  } else
=======
    // Solo acumulamos integral si el error no es gigantesco (evita overshoot
    // inicial)
    if (fabs(error) < 3.0f) { //fabs(error) < 2.0f OK - pero 3.0f más suave
      integral += error * dt;
    }

    // Tu límite de 10.0f está bien, pero recuerda que es "integral acumulada"
    integral = constrain(integral, -20.0f, 60.0f); //-20.0f, 75.0f OK - pero 60.0f más suave
  } else {
>>>>>>> 709c66cf67a90b0d24b2924397561c153b988078
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

  previous_pv = process_variable;
  return output;
}
