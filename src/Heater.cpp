#include "Heater.h"
#include "Logger.h"
#include <Arduino.h>

void Heater::begin() {
  ledcSetup(heatPwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(PWM_HEATER_PIN, heatPwmChannel);

  ledcSetup(coolPwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(PWM_COOLING_PIN, coolPwmChannel);

  pinMode(EN_HEATER_PIN, OUTPUT);
  pinMode(EN_COOLING_PIN, OUTPUT);

  stop();
  logln("Heater/Cooler initialized.");
}

void Heater::setPower(float dutyCycle) {
  dutyCycle = constrain(dutyCycle, -100.0f, 100.0f);

  updateFanLogic(dutyCycle);

  if (dutyCycle > 0.0f) {
    if (dutyCycle > MAX_HEAT_POWER) {
      dutyCycle = MAX_HEAT_POWER;
    } 
    setHeat(dutyCycle);
    return;
  }
  
  if (dutyCycle < 0.0f) {
    if (fabs(dutyCycle) > MAX_COOL_POWER) {
      dutyCycle = -MAX_COOL_POWER;
    }
    setCool(fabs(dutyCycle));
    return;
  }

  stop();
}

void Heater::setHeat(float dutyCycle) {
  dutyCycle = constrain(dutyCycle, 0.0f, MAX_HEAT_POWER);

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
  dutyCycle = constrain(dutyCycle, 0.0f, MAX_COOL_POWER);

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

void Heater::updateFanLogic(float dutyCycle) {
    if (fabs(dutyCycle) < 0.5f) {
        fan.stop();
        return;
    }

    fan.setPower(dutyCycle > 0 ? fan.MIN_FAN_POWER : fan.MAX_FAN_POWER);
}

void Heater::stop() {
  digitalWrite(EN_HEATER_PIN, LOW);
  digitalWrite(EN_COOLING_PIN, LOW);

  ledcWrite(heatPwmChannel, 0);
  ledcWrite(coolPwmChannel, 0);

  fan.stop();
}