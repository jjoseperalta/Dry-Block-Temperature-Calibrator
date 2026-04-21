#ifndef HEATER_H
#define HEATER_H

#include <Arduino.h>
#include "Settings.h"
#include "Fan.h"

extern Settings settings;

const int PWM_HEATER_PIN = 27;
const int EN_HEATER_PIN = 25;
const int PWM_COOLING_PIN = 26;
const int EN_COOLING_PIN = 33;

class Heater {
public:
    Heater(Fan &fanInstance) : fan(fanInstance) {}

    float MAX_HEAT_POWER = 75.0f;
    float MAX_COOL_POWER = 100.0f;

    void begin();
    void setPower(float dutyCycle);
    void setHeat(float dutyCycle);
    void setCool(float dutyCycle);
    void updateFanLogic(float dutyCycle);
    void stop();

private:
    Fan &fan;
    const int heatPwmChannel = 0;
    const int coolPwmChannel = 1;
    const int pwmFrequency = 500; 
    const int pwmResolution = 10;
};

#endif // HEATER_H