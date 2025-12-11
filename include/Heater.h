#ifndef HEATER_H
#define HEATER_H

#include <Arduino.h>

// Pin definitions
const int PWM_HEATER_PIN = 14;
const int PWM_PELTIER_PIN = 27;

class Heater {
public:
    void begin();
    void setHeat(float dutyCycle); // 0.0 to 100.0
    void setCool(float dutyCycle); // 0.0 to 100.0
    void stop();

private:
    const int heatPwmChannel = 0;
    const int coolPwmChannel = 1;
    const int pwmFrequency = 5000;
    const int pwmResolution = 8;
};

#endif // HEATER_H
