#include "PIDController.h"
#include "Logger.h"
#include <Arduino.h>
#include <cmath>

PIDController::PIDController(Settings &settings)
    : settings(settings), currentInput(0.0f), currentOutput(0.0f),
      currentSetpoint(0.0f), lastOutput(0.0f), previousPV(0.0f), firstRun(true),
      currentState(ThermalState::RAMPING),
      lastLoggedState(ThermalState::RAMPING), lastLoggedHeating(true),
      lastLoggedKp(0.0f), lastLoggedKi(0.0f), lastLoggedKd(0.0f),
      myPID(&currentInput, &currentOutput, &currentSetpoint, 1.8f, 0.05f, 1.0f,
            QuickPID::pMode::pOnMeas, QuickPID::dMode::dOnMeas,
            QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct) {

  // QuickPID operará internamente en un rango de 0 a 100 de potencia pura.
  // El signo negativo para enfriamiento lo gestionamos en la salida de
  // compute().
  myPID.SetOutputLimits(0.0f, OUTPUT_MAX);
  myPID.SetMode(QuickPID::Control::automatic);

  logln("========================================");
  logln("     PID CONTROLLER INITIALIZED (V6)");
  logln("========================================");
  logf("  Limits: Min=%.1f%%, Max=%.1f%%\n", OUTPUT_MIN, OUTPUT_MAX);
  logf("  Ramping Thresholds: Heat=%.1f°C, Cool=%.1f°C\n",
       RAMPING_THRESHOLD_HEAT, RAMPING_THRESHOLD_COOL);
  logln("========================================");
}

float PIDController::compute(float setpoint, float input, float dt,
                             bool forceCoolingBrake) {
  currentSetpoint = setpoint;
  currentInput = input;

  if (dt <= 0.0f)
    dt = 1.0f;
  myPID.SetSampleTimeUs((unsigned long)(dt * 1000000.0f));

  float error = setpoint - input;
  float absError = fabs(error);
  bool isHeating = (setpoint > input);

  ThermalState newState = determineThermalState(absError, isHeating);
  applyAdaptiveTunings(newState, isHeating, absError);
  currentState = newState;

  myPID.Compute();

  float rawOutput = currentOutput;
  if (!isHeating) {
    rawOutput = -rawOutput;
  }

  float controlledOutput = applySlewRate(rawOutput, isHeating);

  // PASO 5 MODIFICADO: Le pasamos el flag de enfriamiento forzado
  controlledOutput =
      applyBraking(controlledOutput, absError, isHeating, forceCoolingBrake);

  if (controlledOutput > OUTPUT_MAX)
    controlledOutput = OUTPUT_MAX;
  if (controlledOutput < OUTPUT_MIN)
    controlledOutput = OUTPUT_MIN;

  static uint32_t lastDebugLog = 0;
  if (millis() - lastDebugLog > 5000) {
    lastDebugLog = millis();
    logf("[PID DEBUG] Mode: %s | Err: %.2f | RawPID: %.2f | FinalOut: %.2f\n",
         isHeating ? "HEAT" : "COOL", error, currentOutput, controlledOutput);
  }

  return controlledOutput;
}

void PIDController::reset() {
  currentOutput = 0.0f;
  lastOutput = 0.0f;
  previousPV = 0.0f;
  firstRun = true;
  currentState = ThermalState::RAMPING;
  lastLoggedState = ThermalState::RAMPING;

  myPID.Reset();
  myPID.SetMode(QuickPID::Control::automatic);
}

void PIDController::setPreviousPV(float previousPV) {
  this->previousPV = previousPV;
}

void PIDController::setTunings(float kp, float ki, float kd) {
  myPID.SetTunings(kp, ki, kd);
}

void PIDController::getTunings(float &kp, float &ki, float &kd) {
  kp = myPID.GetKp();
  ki = myPID.GetKi();
  kd = myPID.GetKd();
}

ThermalState PIDController::determineThermalState(float absError,
                                                  bool isHeating) {
  float rampingThreshold =
      isHeating ? RAMPING_THRESHOLD_HEAT : RAMPING_THRESHOLD_COOL;

  if (absError > rampingThreshold) {
    return ThermalState::RAMPING;
  } else if (absError > HOLDING_THRESHOLD) {
    return ThermalState::APPROACHING;
  } else {
    return ThermalState::HOLDING;
  }
}

void PIDController::applyAdaptiveTunings(ThermalState state, bool isHeating,
                                         float absError) {
  float kp, ki, kd;
  float rampingThreshold =
      isHeating ? RAMPING_THRESHOLD_HEAT : RAMPING_THRESHOLD_COOL;

  // Configurar Dirección del Controlador de forma dinámica
  if (isHeating) {
    myPID.SetControllerDirection(QuickPID::Action::direct);
  } else {
    myPID.SetControllerDirection(QuickPID::Action::reverse);
  }

  if (state == ThermalState::RAMPING) {
    if (isHeating) {
      kp = DEFAULT_AGGR_KP_HEAT;
      ki = DEFAULT_AGGR_KI_HEAT;
      kd = DEFAULT_AGGR_KD_HEAT;
    } else {
      kp = DEFAULT_AGGR_KP_COOL;
      ki = DEFAULT_AGGR_KI_COOL;
      kd = DEFAULT_AGGR_KD_COOL;
    }
  } else if (state == ThermalState::APPROACHING) {
    // INTERPOLACIÓN LINEAL: Suaviza la transición de Ramping a Holding para
    // evitar baches térmicos
    float factor =
        (absError - HOLDING_THRESHOLD) / (rampingThreshold - HOLDING_THRESHOLD);
    factor = constrain(factor, 0.0f, 1.0f);

    if (isHeating) {
      kp = DEFAULT_KP_HEAT + (DEFAULT_AGGR_KP_HEAT - DEFAULT_KP_HEAT) * factor;
      ki = DEFAULT_KI_HEAT + (DEFAULT_AGGR_KI_HEAT - DEFAULT_KI_HEAT) * factor;
      kd = DEFAULT_KD_HEAT + (DEFAULT_AGGR_KD_HEAT - DEFAULT_KD_HEAT) * factor;
    } else {
      kp = DEFAULT_KP_COOL + (DEFAULT_AGGR_KP_COOL - DEFAULT_KP_COOL) * factor;
      ki = DEFAULT_KI_COOL + (DEFAULT_AGGR_KI_COOL - DEFAULT_KI_COOL) * factor;
      kd = DEFAULT_KD_COOL + (DEFAULT_AGGR_KD_COOL - DEFAULT_KD_COOL) * factor;
    }
  } else { // ThermalState::HOLDING
    if (isHeating) {
      kp = DEFAULT_KP_HEAT;
      ki = DEFAULT_KI_HEAT;
      kd = DEFAULT_KD_HEAT;
    } else {
      kp = DEFAULT_KP_COOL;
      ki = DEFAULT_KI_COOL;
      kd = DEFAULT_KD_COOL;
    }
  }

  myPID.SetTunings(kp, ki, kd);
  logTuningChange(state, isHeating, kp, ki, kd);
}

float PIDController::applyBraking(float output, float absError, bool isHeating,
                                  bool forceCoolingBrake) {
  float p1 = settings.getCalibrationPoint(0, 'C'); // 1.11°C
  float p2 = settings.getCalibrationPoint(1, 'C'); // 20°C
  float p3 = settings.getCalibrationPoint(2, 'C'); // 38.89°C
  float currentSetpointC = settings.getSetpoint();

  const float TOL = 0.05f;

  // Solo frenar si NO vamos hacia 1.11°C Y (estamos en transición
  // válida: 1.11->20, 20->38, o 38->20)
  bool shouldBrake = (fabsf(currentSetpoint - p1) > TOL) &&
                     ((fabsf(currentSetpoint - p2) < TOL) ||
                      (fabsf(currentSetpoint - p3) < TOL));

  if (!shouldBrake) {
    return output;
  }
  // Distancia de frenado por software a 8.0 grados antes del setpoint
  static constexpr float CUSTOM_COAST_DISTANCE = 9.0f;

  if (absError < CUSTOM_COAST_DISTANCE) {
    float brakeForce = 1.0f;

    // MODIFICADO: Subimos los pisos de fuerza para que no se quede sin energía
    // frente al ventilador
    if (absError < 2.0f) {
      brakeForce = 0.70f; // ANTES: 0.20f -> Subimos al 35% para romper el
                          // estancamiento final
    } else if (absError < 3.0f) {
      brakeForce = 0.80f; // ANTES: 0.40f -> Le damos un 50% de paso para
                          // empujar con ganas
    } else if (absError < 5.0f) {
      brakeForce = 0.90f; // ANTES: 0.60f
    } else if (absError < 9.0f) {
      brakeForce = 0.95f; // Se mantiene igual
    }

    output *= brakeForce;
  }

  return output;
}

// float PIDController::applySlewRate(float output, bool isHeating) {
//   // Si no está calentando, guardamos la salida actual para el próximo ciclo
//   y salimos intactos if (!isHeating) {
//     firstRun = false;
//     lastOutput = output;
//     return output;
//   }

//   // A partir de aquí, el control de Slew Rate aplica SOLO si isHeating es
//   verdadero if (firstRun) {
//     firstRun = false;
//     lastOutput = output;
//     return output;
//   }

//   float outputChange = output - lastOutput;
//   if (fabs(outputChange) > MAX_OUTPUT_CHANGE) {
//     output = lastOutput + (outputChange > 0 ? MAX_OUTPUT_CHANGE :
//     -MAX_OUTPUT_CHANGE);
//   }

//   lastOutput = output;
//   return output;
// }

float PIDController::applySlewRate(float output, bool isHeating) {
  // Aplicar slew rate para AMBAS direcciones
  if (firstRun) {
    firstRun = false;
    lastOutput = output;
    return output;
  }

  float outputChange = output - lastOutput;
  if (fabs(outputChange) > MAX_OUTPUT_CHANGE) {
    output = lastOutput +
             (outputChange > 0 ? MAX_OUTPUT_CHANGE : -MAX_OUTPUT_CHANGE);
  }

  lastOutput = output;
  return output;
}

void PIDController::logTuningChange(ThermalState newState, bool isHeating,
                                    float newKp, float newKi, float newKd) {
  bool stateChanged = (newState != lastLoggedState);
  bool directionChanged = (isHeating != lastLoggedHeating);
  bool tuningsChanged = (fabs(newKp - lastLoggedKp) > 0.01f ||
                         fabs(newKi - lastLoggedKi) > 0.001f ||
                         fabs(newKd - lastLoggedKd) > 0.01f);

  if (stateChanged || directionChanged || tuningsChanged) {
    const char *oldStateStr =
        lastLoggedState == ThermalState::RAMPING
            ? "RAMPING"
            : (lastLoggedState == ThermalState::APPROACHING ? "APPROACHING"
                                                            : "HOLDING");
    const char *newStateStr =
        newState == ThermalState::RAMPING
            ? "RAMPING"
            : (newState == ThermalState::APPROACHING ? "APPROACHING"
                                                     : "HOLDING");
    const char *oldDirStr = lastLoggedHeating ? "HEAT" : "COOL";
    const char *newDirStr = isHeating ? "HEAT" : "COOL";

    if (stateChanged || directionChanged) {
      logf("[PID SWITCH] %s->%s | %s->%s | KP: %.2f->%.2f | KI: %.3f->%.3f | "
           "KD: %.2f->%.2f\n",
           oldStateStr, newStateStr, oldDirStr, newDirStr, lastLoggedKp, newKp,
           lastLoggedKi, newKi, lastLoggedKd, newKd);
    } else if (tuningsChanged && newState != ThermalState::APPROACHING) {
      // Evitamos inundar el log en APPROACHING ya que cambia continuamente por
      // la interpolación lineal
      logf(
          "[PID UPDATE] State: %s | Dir: %s | KP: %.2f | KI: %.3f | KD: %.2f\n",
          newStateStr, newDirStr, newKp, newKi, newKd);
    }

    lastLoggedState = newState;
    lastLoggedHeating = isHeating;
    lastLoggedKp = newKp;
    lastLoggedKi = newKi;
    lastLoggedKd = newKd;
  }
}

void PIDController::printCurrentTunings() {
  float kp = myPID.GetKp();
  float ki = myPID.GetKi();
  float kd = myPID.GetKd();
  logln("========================================");
  logf("  State: %s | Dir: %s\n", getStateString(),
       (currentSetpoint > currentInput) ? "HEATING" : "COOLING");
  logf("  Kp: %.4f | Ki: %.4f | Kd: %.4f\n", kp, ki, kd);
  logln("========================================");
}

void PIDController::setOutputLimits(float min, float max) {
  // Mantenemos la consistencia interna (0 .. Max)
  myPID.SetOutputLimits(0.0f, max);
}

const char *PIDController::getStateString() const {
  switch (currentState) {
  case ThermalState::RAMPING:
    return "RAMPING";
  case ThermalState::APPROACHING:
    return "APPROACHING";
  case ThermalState::HOLDING:
    return "HOLDING";
  default:
    return "UNKNOWN";
  }
}