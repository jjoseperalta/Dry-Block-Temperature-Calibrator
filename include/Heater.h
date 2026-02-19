#ifndef HEATER_H
#define HEATER_H

#include <Arduino.h>
#include "Settings.h"

extern Settings settings;

struct MicroPWMConfig {
  float basePower;   // % potencia base
  float scaleFactor; // factor de escala para potencia variable
  float minPower;    // % potencia mínima efectiva
  float maxPower;    // % potencia máxima efectiva
  uint32_t periodMs; // periodo térmico
  uint32_t onTimeMs; // tiempo encendido
};

const int PWM_HEATER_PIN = 14; 
const int EN_HEATER_PIN = 22; 
const int EN_COOLING_PIN = 21; 
const int PWM_COOLING_PIN = 27;

class Heater {
public:
    void begin();
    void setPower(float dutyCycle, bool fineZone = false);
    void setHeat(float dutyCycle);
    void setCool(float dutyCycle);
    void stop();

private:
    const int heatPwmChannel = 0;
    const int coolPwmChannel = 1;
    const int pwmFrequency = 500; 
    const int pwmResolution = 10;
    MicroPWMConfig microFine;
    bool fineZoneActive = false;
    uint32_t fineZoneStartTime = 0;
};

#endif // HEATER_H