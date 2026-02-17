#include "Heater.h"
#include "Logger.h"
#include <Arduino.h>

void Heater::begin() {
  microFine.basePower = 8.5f;
  microFine.scaleFactor = 0.5f;
  microFine.minPower = 8.0f; // 6.1f para 25Grados
  microFine.maxPower = 15.0f;
  microFine.periodMs = 4000; // 4000 para 25Grados
  microFine.onTimeMs = 400;

  ledcSetup(heatPwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(PWM_HEATER_PIN, heatPwmChannel);

  ledcSetup(coolPwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(PWM_COOLING_PIN, coolPwmChannel);

  pinMode(EN_HEATER_PIN, OUTPUT);
  pinMode(EN_COOLING_PIN, OUTPUT);

  stop();
  logln("Heater/Cooler initialized with PWM Dual (5kHz).");
}

void Heater::setPower(float dutyCycle, bool fineZone) {
  dutyCycle = constrain(dutyCycle, -100.0f, 100.0f);

  const float magnitude = fabs(dutyCycle);
  const float sign = (dutyCycle >= 0) ? 1.0f : -1.0f;

  if (fineZone && magnitude > 0.0f) {
    uint32_t now = millis();
    if (!fineZoneActive) {  // NUEVO: flag para detectar transición
      fineZoneActive = true;
      fineZoneStartTime = now;
    }

    uint32_t phase = (now - fineZoneStartTime) % microFine.periodMs;

    uint32_t dynamicOnTime = (magnitude / 100.0f) * microFine.periodMs;

    if (magnitude > 0.01f)
      if (dynamicOnTime < 250)
        dynamicOnTime = 250;

    if (dynamicOnTime > microFine.periodMs)
      dynamicOnTime = microFine.periodMs;

    if (phase < dynamicOnTime) {
      float powerLevel = 7.0f + (magnitude * 0.3f);

      if (powerLevel > 15.0f)
        powerLevel = 15.0f;

      dutyCycle = powerLevel * sign;
    } else {
      dutyCycle = 0.0f;
    }
  } else {
    fineZoneActive = false; // NUEVO: reset flag al salir de zona fina
    // fineZoneStartTime = 0;
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

// void Heater::setPower(float dutyCycle, bool fineZone) {
//   dutyCycle = constrain(dutyCycle, -100.0f, 100.0f);

//   const float magnitude = fabs(dutyCycle);
//   const float sign = (dutyCycle >= 0.0f) ? 1.0f : -1.0f;

//   static bool wasFineZone = false;
//   static uint32_t t0 = 0;

//   if (fineZone && magnitude > 0.0f) {
//     uint32_t now = millis();

//     if (!wasFineZone)
//       t0 = now;

//     uint32_t phase = (now - t0) % microFine.periodMs;

//     uint32_t onTime = (uint32_t)((magnitude * microFine.periodMs) / 100.0f);
//     onTime = constrain(onTime, 250UL, microFine.periodMs);

//     if (phase < onTime) {
//       float powerLevel =
//           microFine.basePower + (magnitude * microFine.scaleFactor);
//       powerLevel =
//           constrain(powerLevel, microFine.basePower, microFine.maxPower);

//       dutyCycle = powerLevel * sign;
//     } else {
//       dutyCycle = 0.0f;
//     }

//     wasFineZone = true;
//   } else {
//     wasFineZone = false;
//     t0 = 0;
//   }

//   // Despacho final
//   if (dutyCycle > 0.0f)
//     setHeat(dutyCycle);
//   else if (dutyCycle < 0.0f)
//     setCool(-dutyCycle);
//   else
//     stop();
// }

void Heater::setHeat(float dutyCycle) {
  dutyCycle = constrain(dutyCycle, 0.0f, 100.0f);

  const uint32_t maxDuty = (1 << pwmResolution) - 1;
  const uint32_t duty = (uint32_t)((dutyCycle * maxDuty) / 100.0f);

  if (duty == 0) {
    stop();
    return;
  }

  digitalWrite(EN_COOLING_PIN, HIGH);
  digitalWrite(EN_HEATER_PIN, HIGH);

  ledcWrite(coolPwmChannel, 0);
  ledcWrite(heatPwmChannel, duty);

  if (duty > 0)
    logf("Heater set to %.2f%% power (Duty: %u)\n", dutyCycle, duty);
}

void Heater::setCool(float dutyCycle) {
  dutyCycle = constrain(dutyCycle, 0.0f, 100.0f);

  const uint32_t maxDuty = (1 << pwmResolution) - 1;
  const uint32_t duty = (uint32_t)((dutyCycle * maxDuty) / 100.0f);

  if (duty == 0) {
    stop();
    return;
  }

  digitalWrite(EN_HEATER_PIN, HIGH);
  digitalWrite(EN_COOLING_PIN, HIGH);

  ledcWrite(heatPwmChannel, 0);
  ledcWrite(coolPwmChannel, duty);

  if (duty > 0)
    logf("Cooling set to %.2f%% power (Duty: %u)\n", dutyCycle, duty);
}

void Heater::stop() {
  ledcWrite(heatPwmChannel, 0);
  ledcWrite(coolPwmChannel, 0);

  digitalWrite(EN_HEATER_PIN, LOW);
  digitalWrite(EN_COOLING_PIN, LOW);

  // logln("Heater/Cooler stopped.");
}
