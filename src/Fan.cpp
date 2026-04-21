#include "Fan.h"
#include "Logger.h"

void Fan::begin() {
  ledcSetup(fanPwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(FAN_PWM_PIN, fanPwmChannel);

  stop();
  logln("Fan initialized.");
}

void Fan::setPower(float dutyCycle) {
  // Limitar entrada
  dutyCycle = constrain(dutyCycle, 0.0f, 100.0f);

  // Si la potencia pedida es muy baja, apagar
  if (dutyCycle < 1.0f) {
    stop();
    return;
  }

  // Aplicar umbral mínimo: si pide 10%, forzar al mínimo de arranque (ej. 35%)
  // Esto evita que el ventilador reciba voltaje insuficiente para girar.
  if (dutyCycle < MIN_FAN_POWER) {
    dutyCycle = MIN_FAN_POWER;
  }

  currentPower = dutyCycle;
  applyPWM(currentPower);
}

void Fan::applyPWM(float dutyCycle) {

  uint32_t maxDuty = (1 << pwmResolution) - 1; // 1023
  uint32_t duty = (uint32_t)((dutyCycle * maxDuty) / 100.0f);

  ledcWrite(fanPwmChannel, duty);
  logf("Fan PWM set to %.2f%% (Duty: %u)\n", dutyCycle, duty);
}

void Fan::stop() {
  currentPower = 0.0f;
  ledcWrite(fanPwmChannel, 0);
  // logln("Fan: Stopped");
}