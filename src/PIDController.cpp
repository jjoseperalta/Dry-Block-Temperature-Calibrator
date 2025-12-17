#include "PIDController.h"
#include "Logger.h"
#include <Arduino.h>
#include <cmath>

PIDController::PIDController(Settings &settings) : settings(settings) {
  // integralLimit = 0.0f;
  reset();
}

void PIDController::reset() {
  integral = 0.0f;
  // previous_error = 0.0f;
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
    // CALENTAR (más agresivo)
    g.kp = kp * (fineZone ? 0.5f : 1.5f); //(fineZone ? 0.6f : 1.2f);
    g.ti = ti * (fineZone ? 2.0f : 0.7f);
    g.td = td * 0.6f; //(fineZone ? 0.3f : 0.0f);
  } else {
    // ENFRIAR (más lento, más amortiguado)
    g.kp = kp * (fineZone ? 0.35f : 0.8f);
    g.ti = ti * (fineZone ? 2.0f : 1.0f);
    g.td = td * 1.3f; //(fineZone ? 1.5f : 1.3f);
  }

  return g;
}

float PIDController::calculate(float setpoint, float process_variable,
                               float dt) {
  if (dt <= 0.0f)
    dt = 0.001f;

  float error = setpoint - process_variable;
  float absError = fabsf(error);

  // --------------------------------------------------
  // ZONAS con histéresis (industrial)
  // --------------------------------------------------
  const float ZONE_ENTER_FINE = 0.25f;
  const float ZONE_EXIT_FINE = 0.50f;

  bool wasFine = inFineZone;

  if (inFineZone) {
    if (absError > ZONE_EXIT_FINE)
      inFineZone = false;
  } else {
    if (absError < ZONE_ENTER_FINE)
      inFineZone = true;
  }

  // --------------------------------------------------
  // Descarga parcial de la integral al cruzar setpoint
  // --------------------------------------------------
  bool heatingNow = (error >= 0.0f);

  if (lastHeating && !heatingNow) {
    // Pasamos de calentar a enfriar (cruce del setpoint)
    integral *= 0.3f; // descarga suave, NO reset total
  }

  lastHeating = heatingNow;

  // Velocidad de cambio térmico (°C/s)
  float tempRate = (process_variable - previous_pv) / dt;

  // Si nos acercamos rápido al setpoint, frenar la integral
  // if ((error > 0 && tempRate > 0.02f) || (error < 0 && tempRate < -0.02f)) {
  //   // Estamos yendo hacia el setpoint con inercia
  //   // NO integrar
  // } else {
  //   integral += error * dt;
  // }

  integral = constrain(integral, -10.0f, 10.0f);

  // Reset integral SOLO al entrar en zona fina
  if (inFineZone) {
    integral = constrain(integral, -8.0f, 8.0f);
  }

  // --------------------------------------------------
  // Selección de ganancias
  // --------------------------------------------------
  PIDGains g = computeGains(error, inFineZone);

  // --- 1. Término Proporcional (P) ---
  float p_out = g.kp * error;

  // --- 2. Término Derivativo (D) ---
  // Derivativo en la medición (PV) para evitar picos en el cambio de setpoint
  float derivative = (process_variable - previous_pv) / dt;
  float d_out =
      -g.kp * g.td * derivative; // Signo negativo para frenar el calentamiento

  // --- 3. Término Integral (I) con Anti-Windup Clamping ---
  float i_out = 0.0f;
  float tentative_output =
      p_out + d_out; // Salida P+D antes de incluir la Integral

  if (g.ti > 1e-6f) {
    // Cálculo de la acción I (Ki * integral)
    float kp_over_ti = g.kp / g.ti;

    // 1. Determinar si la salida P+D+I estaría saturada
    // Usamos el estado actual de la integral para ver si contribuye a la
    // saturación.
    float tentative_output_full = tentative_output + (kp_over_ti * integral);

    bool output_clamped = false;
    if (tentative_output_full > 100.0f && error > 0.0f) {
      output_clamped = true;
    } else if (tentative_output_full < -100.0f && error < 0.0f) {
      output_clamped = true;
    }

    // float minHeat = settings.getMinHeatPower();
    // float minCool = settings.getMinCoolPower();

    // float minHeatEffective = inFineZone ? (minHeat + 0.8f) : minHeat;

    // bool actuatorEffective =
    //     (tentative_output > minHeatEffective) || (tentative_output <
    //     -minCool);

    if (!output_clamped) {
      if (!inFineZone || fabs(error) < 0.3f) {
        integral += error * dt;
      }
    }

    // 3. Calcular la salida final del término I
    i_out = kp_over_ti * integral;
  } else {
    // Si Ti es cero o muy pequeño, la integral no se usa y se limpia
    integral = 0.0f;
  }

  // 1) Calcular output PID
  float output = p_out + i_out + d_out;

  // 2) Integral lenta de HOLD
  // if (inFineZone && fabsf(error) < 0.6f) {
  //   const float I_HOLD_GAIN = 0.035f;
  //   integral += error * dt * I_HOLD_GAIN;
  //   integral = constrain(integral, -5.0f, 5.0f);
  // }

  // 3) Recalcular i_out tras modificar integral
  // i_out = (g.kp / g.ti) * integral;
  // output = p_out + i_out + d_out;

  // 4) Bloquear cooling SOLO cerca del setpoint real (no en cooling profundo)
if (inFineZone && output < 0.0f && process_variable < setpoint + 0.2f) {
    output = 0.0f;
}

  // 5) Clamp final
  output = constrain(output, -100.0f, 100.0f);

  previous_pv = process_variable;

  return output;
}