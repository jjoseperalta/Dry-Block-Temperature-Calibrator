#ifndef FAN_H
#define FAN_H

#include <Arduino.h>
#include "Settings.h"

const int FAN_PWM_PIN = 33;

enum class FanMode {
    OFF,      // Ventilador apagado
    HEATING,  // Calentando: rampa hacia 20%
    COOLING   // Enfriando: rampa hacia 100%
};

class Fan {
public:
    void begin();
    void setMode(FanMode mode);              // Cambia modo (inicia rampa)
    void update();                           // ← NUEVO: Actualiza rampa cada ciclo
    void setPower(float dutyCycle);          // Control manual directo (sin rampa)
    void stop();
    FanMode getCurrentMode() const;
    float getCurrentPower() const;           // ← NUEVO: Lee potencia actual
    
private:
    const int fanPwmChannel = 5;
    const int pwmFrequency = 20000;
    const int pwmResolution = 10;
    const float MIN_FAN_POWER = 40.0f; // El mínimo para que AMBOS giren
    
    // Parámetros de rampa
    const float RAMPING_TIME_MS = 2000.0f;  // Tiempo para hacer rampa (2 segundos)
    
    FanMode currentMode = FanMode::OFF;
    float currentPower = 0.0f;               // Potencia actual (0-100%)
    float targetPower = 0.0f;                // Potencia objetivo
    uint32_t rampStartTime = 0;              // Cuándo comenzó la rampa
    bool isRamping = false;                  // ¿Está en rampa?
    
    void applyPWM(float dutyCycle);          // ← NUEVO: Aplica PWM sin rampa
};

#endif // FAN_H