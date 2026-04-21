#ifndef FAN_H
#define FAN_H

#include <Arduino.h>

const int FAN_PWM_PIN = 14;

class Fan {
public:
    void begin();
    
    /**
     * @brief Establece la potencia del ventilador de 0 a 100.
     * Maneja internamente el umbral mínimo (MIN_FAN_POWER).
     */
    void setPower(float dutyCycle);
    void stop();
    // Valor mínimo para vencer la inercia de los ventiladores
    float MIN_FAN_POWER = 42.0f;
    float MAX_FAN_POWER = 100.0f;
    float getCurrentPower() const { return currentPower; }
    
private:
    const int fanPwmChannel = 5;
    const int pwmFrequency = 20000;
    const int pwmResolution = 10; 
    
    float currentPower = 0.0f;
    void applyPWM(float dutyCycle);
};

#endif