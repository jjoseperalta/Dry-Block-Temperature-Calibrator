#include "Fan.h"
#include "Logger.h"

void Fan::begin() {
  ledcSetup(fanPwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(FAN_PWM_PIN, fanPwmChannel);

  stop();
  logln("Fan initialized with PWM (5kHz) and ramping.");
}

void Fan::setMode(FanMode mode) {
  if (mode == currentMode) return;

  currentMode = mode;
  rampStartTime = millis();
  isRamping = true;

  switch (mode) {
  case FanMode::OFF:
    targetPower = 0.0f;
    logln("Fan: Ramping to OFF");
    break;

  case FanMode::HEATING:
    targetPower = MIN_FAN_POWER;
    logf("Fan: Ramping to HEATING (target: %.1f%%)\n", targetPower);
    break;

  case FanMode::COOLING:
    targetPower = 100.0f;
    logf("Fan: Ramping to COOLING (target: %.1f%%)\n", targetPower);
    break;
  }
}

void Fan::update() {
  if (!isRamping)
    return;

  uint32_t now = millis();
  uint32_t elapsedTime = now - rampStartTime;
  float progress = (float)elapsedTime / RAMPING_TIME_MS;

  if (progress >= 1.0f) {
    currentPower = targetPower;
    isRamping = false;
  } else {
    // Rampa suave con easing (curva)
    // Usa función easeInOutQuad para suavidad
    float eased = progress < 0.5f ? 2 * progress * progress
                                  : 1 - pow(-2 * progress + 2, 2) / 2;

    currentPower = currentPower + (targetPower - currentPower) * eased;
  }

  applyPWM(currentPower);
}

void Fan::setPower(float dutyCycle) {
  // Control DIRECTO sin rampa (para emergencias o control manual)
  dutyCycle = constrain(dutyCycle, 0.0f, 100.0f);
  currentPower = dutyCycle;
  targetPower = dutyCycle;
  isRamping = false; // Cancela cualquier rampa en curso

  applyPWM(dutyCycle);
  logf("Fan: Direct power set to %.1f%%\n", dutyCycle);
}

void Fan::applyPWM(float dutyCycle) {
  dutyCycle = constrain(dutyCycle, 0.0f, 100.0f);

  if (dutyCycle > 0.0f && dutyCycle < MIN_FAN_POWER) {
    dutyCycle = MIN_FAN_POWER;
  } else if (dutyCycle <= 0.0f) {
    dutyCycle = 0.0f;
  }

  uint32_t maxDuty = (1 << pwmResolution) - 1;
  uint32_t duty = (uint32_t)((dutyCycle * maxDuty) / 100.0f);

  ledcWrite(fanPwmChannel, duty);
}

void Fan::stop() {
  currentMode = FanMode::OFF;
  currentPower = 0.0f;
  targetPower = 0.0f;
  isRamping = false;
  ledcWrite(fanPwmChannel, 0);
}

FanMode Fan::getCurrentMode() const { return currentMode; }

float Fan::getCurrentPower() const { return currentPower; }