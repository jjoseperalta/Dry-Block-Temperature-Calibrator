#ifndef HEATER_H
#define HEATER_H

#include <Arduino.h>

// Pines y Canales (Sin cambios en tu definición)
const int PWM_HEATER_PIN = 14; 
const int EN_HEATER_PIN = 22; 
const int EN_COOLING_PIN = 21; 
const int PWM_COOLING_PIN = 27;

class Heater {
public:
    void begin();
    void setHeat(float dutyCycle); // 0.0 to 100.0 (PWM en RPWM)
    void setCool(float dutyCycle); // 0.0 to 100.0 (PWM en LPWM)
    void stop();

private:
    const int heatPwmChannel = 0;
    const int coolPwmChannel = 1;
    // ¡Ajuste de Frecuencia Crítico! 1Hz es muy bajo. Usaremos 2kHz.
    const int pwmFrequency = 5000; 
    const int pwmResolution = 8;
};

#endif // HEATER_H