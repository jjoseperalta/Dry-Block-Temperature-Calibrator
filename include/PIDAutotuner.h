#ifndef PID_AUTOTUNER_H
#define PID_AUTOTUNER_H

#include "Heater.h"
#include "Sensors.h"
#include <Arduino.h>

class PIDAutotuner {
public:
    enum class State { IDLE, OSCILLATING, CALCULATING, FINISHED, ERROR };

    PIDAutotuner(Heater &h, Sensors &s);
    void start(float setpoint, float amplitude);
    void update(float currentTemp);
    bool isFinished() { return state == State::FINISHED; }
    
    // Resultados finales
    float getKp() { return kp; }
    float getTi() { return ti; }
    float getTd() { return td; }

private:
    Heater &heater;
    Sensors &sensors;
    State state = State::IDLE;

    float targetSetpoint;
    float relayAmplitude;
    float kp, ti, td;

    // Variables de medición
    float peakTemp = 0.0f;
    float valleyTemp = 0.0f;
    uint32_t lastCrossingTime = 0;
    float period = 0.0f;
    int cycles = 0;
};

#endif