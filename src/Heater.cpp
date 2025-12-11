#include "Heater.h"

#include <Arduino.h>

void Heater::begin() {
    ledcSetup(heatPwmChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(PWM_HEATER_PIN, heatPwmChannel);

    ledcSetup(coolPwmChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(PWM_PELTIER_PIN, coolPwmChannel);

    stop();
}

void Heater::setHeat(float dutyCycle) {
    if (dutyCycle < 0) dutyCycle = 0;
    if (dutyCycle > 100) dutyCycle = 100;
    uint32_t maxDuty = (1 << pwmResolution) - 1;
    uint32_t duty = (uint32_t)((dutyCycle / 100.0f) * (float)maxDuty);
    ledcWrite(heatPwmChannel, duty);
    ledcWrite(coolPwmChannel, 0);
}

void Heater::setCool(float dutyCycle) {
    if (dutyCycle < 0) dutyCycle = 0;
    if (dutyCycle > 100) dutyCycle = 100;
    uint32_t maxDuty = (1 << pwmResolution) - 1;
    uint32_t duty = (uint32_t)((dutyCycle / 100.0f) * (float)maxDuty);
    ledcWrite(coolPwmChannel, duty);
    ledcWrite(heatPwmChannel, 0);
}

void Heater::stop() {
    ledcWrite(heatPwmChannel, 0);
    ledcWrite(coolPwmChannel, 0);
}
