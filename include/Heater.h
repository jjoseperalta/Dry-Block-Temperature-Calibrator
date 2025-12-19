#ifndef HEATER_H
#define HEATER_H

#include <Arduino.h>
#include "Settings.h"

extern Settings settings;

struct MicroPWMConfig {
  float minPower;        // % potencia mínima efectiva
  uint32_t periodMs;     // periodo térmico
  uint32_t onTimeMs;     // tiempo encendido
};

// Pines y Canales (Sin cambios en tu definición)
const int PWM_HEATER_PIN = 14; 
const int EN_HEATER_PIN = 22; 
const int EN_COOLING_PIN = 21; 
const int PWM_COOLING_PIN = 27;

class Heater {
public:
    void begin();
    void setPower(float dutyCycle, bool fineZone = false); // -100.0 to 100.0
    float applyMinimumEffectivePower(float dutyCycle, bool fineZone = false);
    void setHeat(float dutyCycle); // 0.0 to 100.0 (PWM en RPWM)
    void setCool(float dutyCycle); // 0.0 to 100.0 (PWM en LPWM)
    void stop();
    void hold(float temp, float setpoint);

private:
    const int heatPwmChannel = 0;
    const int coolPwmChannel = 1;
    const int pwmFrequency = 500; 
    const int pwmResolution = 10;
    MicroPWMConfig microFine;
};

#endif // HEATER_H