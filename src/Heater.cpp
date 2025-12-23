#include "Heater.h"
#include "Logger.h"
#include <Arduino.h>

void Heater::begin() {
  microFine.minPower = 15.0f; // 6.1f para 25Grados
  microFine.periodMs = 3000;  // 4000 para 25Grados
  microFine.onTimeMs = 400;

  ledcSetup(heatPwmChannel, pwmFrequency,
            pwmResolution); // Canal 0 para Calentar (Pin 14)
  ledcAttachPin(PWM_HEATER_PIN, heatPwmChannel);

  ledcSetup(coolPwmChannel, pwmFrequency,
            pwmResolution); // Canal 1 para Enfriar (Pin 27)
  ledcAttachPin(PWM_COOLING_PIN, coolPwmChannel);

  pinMode(EN_HEATER_PIN, OUTPUT);
  pinMode(EN_COOLING_PIN, OUTPUT);

  stop();
  logln("Heater/Cooler initialized with PWM Dual (5kHz).");
}

void Heater::setPower(float dutyCycle, bool fineZone) {
  dutyCycle = constrain(dutyCycle, -100.0f, 100.0f);
  float magnitude = fabs(dutyCycle);
  float sign = (dutyCycle >= 0) ? 1.0f : -1.0f;

  if (fineZone && magnitude > 0.0f) {
    static uint32_t t0 = 0;
    uint32_t now = millis();
    if (t0 == 0)
      t0 = now;

    uint32_t phase = (now - t0) % microFine.periodMs;

    // --- CÁLCULO DINÁMICO MEJORADO ---
    uint32_t dynamicOnTime = (magnitude / 100.0f) * microFine.periodMs;

    // Reducimos el mínimo de 600ms a 400ms para permitir ajustes más finos
    if (magnitude > 0.01f)
      if (dynamicOnTime < 250)
        dynamicOnTime = 250;

    if (dynamicOnTime > microFine.periodMs)
      dynamicOnTime = microFine.periodMs;

    if (phase < dynamicOnTime) {
      // En lugar de un valor fijo (7 o 12), usamos una base mínima
      // y le sumamos un poquito de la magnitud que pide el PID.
      // Si el PID pide mucho (setpoint alto), la potencia base sube.
      float powerLevel = 7.0f + (magnitude * 0.3f);

      // Limitemos el powerLevel para que no se vuelva loco
      if (powerLevel > 15.0f)
        powerLevel = 15.0f;
      dutyCycle = powerLevel * sign;
    } else {
      dutyCycle = 0.0f;
    }
  }

  // Despacho de potencia
  if (dutyCycle > 0.0f) {
    setHeat(dutyCycle);
  } else if (dutyCycle < 0.0f) {
    setCool(fabs(dutyCycle));
  } else {
    stop();
  }
}

float Heater::applyMinimumEffectivePower(float dutyCycle, bool fineZone) {
  const float MIN_HEAT_COARSE = settings.getMinHeatPower(); // ej. 6.5 %
  const float MIN_HEAT_FINE = 4.5f;                         // empezar aquí
  const float MIN_COOL = settings.getMinCoolPower();        // ej. 10 %

  float minHeat = fineZone ? MIN_HEAT_FINE : MIN_HEAT_COARSE;

  if (dutyCycle > 0.0f && dutyCycle < minHeat)
    return minHeat;

  if (dutyCycle < 0.0f && dutyCycle > -MIN_COOL)
    return -MIN_COOL;

  return dutyCycle;
}

void Heater::setHeat(float dutyCycle) {
  dutyCycle = constrain(dutyCycle, 0.0f, 100.0f);

  uint32_t maxDuty = (1 << pwmResolution) - 1;

  uint32_t duty = 0;

  if (dutyCycle > 0.0f) {
    float linear_duty = (dutyCycle / 100.0f) * (float)maxDuty;
    duty = (uint32_t)linear_duty;
  }

  if (duty > maxDuty)
    duty = maxDuty; // Cap final (solo por seguridad)

  digitalWrite(EN_HEATER_PIN, HIGH);
  digitalWrite(EN_COOLING_PIN, HIGH);

  // MUTUA EXCLUSIÓN: Asegurar que el lado de Enfriamiento esté en 0
  ledcWrite(coolPwmChannel, 0);

  if (duty > 0)
    logf("Heater set to %.2f%% power (Duty: %u)\n", dutyCycle, duty);

  // Aplicar PWM al lado de Calentamiento
  ledcWrite(heatPwmChannel, duty);
}

void Heater::setCool(float dutyCycle) {
  dutyCycle = constrain(dutyCycle, 0.0f, 100.0f);

  uint32_t maxDuty = (1 << pwmResolution) - 1;

  uint32_t duty = 0;

  if (dutyCycle > 0.0f) {
    float linear_duty = (dutyCycle / 100.0f) * (float)maxDuty;
    duty = (uint32_t)linear_duty;
  }

  if (duty > maxDuty)
    duty = maxDuty;

  digitalWrite(EN_HEATER_PIN, HIGH);
  digitalWrite(EN_COOLING_PIN, HIGH);

  // MUTUA EXCLUSIÓN: Asegurar que el lado de Calentamiento esté en 0
  ledcWrite(heatPwmChannel, 0);

  if (duty > 0)
    logf("Cooling set to %.2f%% power (Duty: %u)\n", dutyCycle, duty);

  // Aplicar PWM al lado de Enfriamiento
  ledcWrite(coolPwmChannel, duty);
}

void Heater::stop() {
  ledcWrite(heatPwmChannel, 0);
  ledcWrite(coolPwmChannel, 0);

  digitalWrite(EN_HEATER_PIN, LOW);
  digitalWrite(EN_COOLING_PIN, LOW);

  // logln("Heater/Cooler stopped.");
}

void Heater::hold(float temp, float setpoint) {
  static float lastTemp = temp;
  static float peakTemp = 0; // Para memorizar el valor del overshoot
  static uint32_t lastPulseTime = 0;
  static uint32_t lastSampleTime = 0;

  uint32_t now = millis();
  float dt = (now - lastSampleTime) / 1000.0f;
  if (dt < 0.1f)
    return;

  float dTdt = (temp - lastTemp) / dt;

  // 1️⃣ DETECTAR EL PICO (OVERSHOOT)
  // Si veníamos subiendo y ahora empezamos a bajar, registramos el pico
  if (temp > setpoint && dTdt < 0 && temp > peakTemp) {
    peakTemp = temp;
  }

  // 2️⃣ REGLA DE SEGURIDAD
  if (temp > (setpoint + 0.5f)) { // Demasiado arriba, no hacer nada
    this->stop();
    lastTemp = temp;
    lastSampleTime = now;
    return;
  }

  // 3️⃣ EVALUACIÓN DESDE EL DESCENSO
  // Si ya estamos bajando desde el pico hacia el setpoint
  if (temp > setpoint && dTdt < -0.001f) {

    // Calculamos qué tan cerca estamos del "suelo" (setpoint)
    // Entre más cerca del setpoint, más "nervioso" debe ponerse el freno
    float distanceToFloor = temp - setpoint;

    if (now - lastPulseTime >= 1500) { // Pulsos espaciados para no recalentar

      // Potencia proporcional a la velocidad de caída y cercanía al suelo
      // A menor distancia, mayor necesidad de frenar
      float brakeForce = fabs(dTdt) * 20.0f;
      float proximityBonus =
          (0.2f / (distanceToFloor + 0.05f)); // Sube al acercarse a SP

      float dynamicHold = 4.0f + brakeForce + proximityBonus;
      dynamicHold = constrain(dynamicHold, 5.0f, 15.0f);

      this->setHeat(dynamicHold);
      lastPulseTime = now;
      logf("!!! FRENO ACTIVO !!! Temp: %.2f | Dist: %.2f | Pot: %.1f\n", temp,
           distanceToFloor, dynamicHold);
    }
  }

  // 4️⃣ RESET DEL PICO
  if (temp <= setpoint) {
    peakTemp = 0; // Reiniciar para el siguiente ciclo
  }

  lastTemp = temp;
  lastSampleTime = now;
}