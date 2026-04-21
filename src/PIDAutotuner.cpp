#include "PIDAutotuner.h"
#include "Logger.h"
#include <Arduino.h>

PIDAutotuner::PIDAutotuner(Heater &h, Sensors &s) : heater(h), sensors(s) {}

void PIDAutotuner::start(float setpoint, float amplitude) {
  this->targetSetpoint = setpoint;
  this->relayAmplitude = amplitude;
  this->state = State::OSCILLATING;
  this->cycles = 0;
  this->lastCrossingTime = millis();
}

void PIDAutotuner::update(float currentTemp) {
  if (state != State::OSCILLATING)
    return;

  // Lógica de Relé: Si está por debajo, calienta; si está por encima, enfría
  if (currentTemp < targetSetpoint) {
    heater.setPower(relayAmplitude);
  } else {
    heater.setPower(-relayAmplitude);
  }

  // Detectar cruces por cero (cruce del setpoint)
  static bool wasBelow = true;
  bool isBelow = (currentTemp < targetSetpoint);

  if (wasBelow != isBelow) {
    // Cruce detectado
    uint32_t now = millis();
    period = (now - lastCrossingTime) * 2.0f; // Periodo de oscilación Tu
    lastCrossingTime = now;
    cycles++;
    wasBelow = isBelow;

    if (cycles >=
        6) { // Necesitamos 3 ciclos completos (6 cruces) para medir bien
      state = State::CALCULATING;
    }
  }

  if (state == State::CALCULATING) {
    float A = (peakTemp - valleyTemp) / 2.0f;

    // PROTECCIÓN: Si la amplitud es casi cero, abortamos
    if (A < 0.05f) {
      logln("ERROR: Autotune fallido. Amplitud muy baja, aumenta "
            "relayAmplitude.");
      state = State::ERROR;
      heater.stop();
      return;
    }

    float Ku = (4.0f * relayAmplitude) / (PI * A);
    float Tu = period / 1000.0f;

    // Ajuste Ziegler-Nichols
    kp = 0.6f * Ku;
    ti = 0.5f * Tu;
    td = 0.125f * Tu;

    state = State::FINISHED;
    heater.stop();
  }
}