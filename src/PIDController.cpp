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

  // if (heating) {
  //   g.kp = kp * (fineZone ? 0.5f : 1.5f);
  //   g.ti = ti * (fineZone ? 2.0f : 0.7f);
  //   g.td = td * 0.6f;
  // } else {
  //   g.kp = kp * (fineZone ? 0.6f : 1.2f);
  //   g.ti = ti * (fineZone ? 2.5f : 0.8f);
  //   g.td = td * (fineZone ? 1.2f : 0.8f);
  // }
  if (heating) {
    // CALENTAR: Agresivo para llegar, predictivo para frenar
    g.kp = kp * (fineZone ? 0.8f : 1.5f);
    g.ti = ti * (fineZone ? 0.6f : 0.7f); // Ti más pequeña = Integral más rápida
    g.td = td * (fineZone ? 0.3f : 0.0f); // Td solo en zona fina para frenar inercia
  } else {
    // ENFRIAR: Suave en potencia, pero muy alto en frenado (D)
    g.kp = kp * (fineZone ? 0.4f : 1.2f);
    g.ti = ti * (fineZone ? 1.5f : 0.8f);
    g.td = td * (fineZone ? 2.0f : 1.0f); // D muy alta para no pasarse enfriando
  }

  return g;
}

// float PIDController::calculate(float setpoint, float process_variable,
//                                float dt) {
//   if (dt <= 0.0f)
//     dt = 0.001f;

//   float error = setpoint - process_variable;
//   float absError = fabsf(error);

//   // Inicialización para el primer ciclo de ejecución
//   static bool firstRun = true;
//   if (firstRun) {
//     lastError = error;
//     previous_pv = process_variable;
//     firstRun = false;
//   }

//   const float ZONE_ENTER_FINE = 0.15f;
//   const float ZONE_EXIT_FINE = 0.60f;

//   if (inFineZone) {
//     if (absError > ZONE_EXIT_FINE)
//       inFineZone = false;
//   } else {
//     if (absError < ZONE_ENTER_FINE)
//       inFineZone = true;
//   }

//   if (!inFineZone && ((lastError > 0.0f && error <= 0.0f) ||
//                       (lastError < 0.0f && error >= 0.0f))) {
//     integral *= 0.3f;
//   }
//   lastError = error;

//   float integralLimit = inFineZone ? 8.0f : 10.0f;
//   integral = constrain(integral, -integralLimit, integralLimit);

//   // Selección de ganancias
//   PIDGains g = computeGains(error, inFineZone);

//   // --- 1. Término Proporcional (P) ---
//   float p_out = g.kp * error;

//   // --- 2. Término Derivativo (D) ---
//   float derivative = (process_variable - previous_pv) / dt;
//   float d_out =
//       -g.kp * g.td * derivative; // Signo negativo para frenar el
//       calentamiento

//   // --- 3. Término Integral (I) con Anti-Windup Clamping ---
//   float i_out = 0.0f;
//   if (g.ti > 1e-6f) {
//     float kp_over_ti = g.kp / g.ti;

//     float tentative_output_full = p_out + d_out + (kp_over_ti * integral);

//     bool output_clamped = false;
//     if (tentative_output_full > 100.0f && error > 0.0f) {
//       output_clamped = true;
//     } else if (tentative_output_full < -100.0f && error < 0.0f) {
//       output_clamped = true;
//     }

//     if (!output_clamped) {
//       if (!inFineZone) {
//         integral += error * dt;
//       }
//     }

//     i_out = kp_over_ti * integral;
//   }

//   // 1) Calcular output PID
//   float output = p_out + i_out + d_out;

//   if (inFineZone) {
//     // Si falta calor (error > 0) pero el PID intenta enfriar (output < 0)
//     cerca
//     // del setpoint
//     if (error > 0.0f && output < 0.0f && process_variable > setpoint - 0.2f)
//     {
//       output = 0.0f;
//     }
//     // Si falta frío (error < 0) pero el PID intenta calentar (output > 0)
//     cerca
//     // del setpoint
//     else if (error < 0.0f && output > 0.0f &&
//              process_variable < setpoint + 0.2f) {
//       output = 0.0f;
//     }
//   }

//   // Limitador de enfriamiento agresivo cerca del setpoint
//   if (output < 0.0f && fabs(error) < 1.0f) {
//     output = max(output, -30.0f);
//   }

//   previous_pv = process_variable;

//   return constrain(output, -100.0f, 100.0f);
// }

float PIDController::calculate(float setpoint, float process_variable,
                               float dt) {
  if (dt <= 0.0f)
    dt = 0.001f;

  // --- Error ---
  float error = setpoint - process_variable;

  // --- Inicialización segura ---
  static bool firstRun = true;
  if (firstRun) {
    previous_pv = process_variable;
    integral = 0.0f;
    firstRun = false;
  }

  // --- Ganancias base (SIN zonas) ---
  PIDGains g = computeGains(error, false); // false = sin zona fina

  // --- Proporcional ---
  float p_out = g.kp * error;

  // --- Derivada sobre la medición ---
  float derivative = (process_variable - previous_pv) / dt;
  float d_out = -g.kp * g.td * derivative;

  // --- Integral con anti-windup por clamping ---
  if (g.ti > 1e-6f) {
    float ki = g.kp / g.ti;
    integral += error * dt;

    // Límite duro de integral (simple y estable)
    integral = constrain(integral, -10.0f, 10.0f);

    // Aplicar integral
    // (se suma abajo)
  } else {
    integral = 0.0f;
  }

  float i_out = (g.ti > 1e-6f) ? (g.kp / g.ti) * integral : 0.0f;

  // --- Salida PID ---
  float output = p_out + i_out + d_out;

  // --- Clamp final ---
  output = constrain(output, -100.0f, 100.0f);

  // --- Actualizar estado ---
  previous_pv = process_variable;

  return output;
}
