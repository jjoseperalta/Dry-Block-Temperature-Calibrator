#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Settings.h"

class PIDController {
public:
    PIDController(Settings& settings);
    // dt: elapsed time in seconds since last calculation
    float calculate(float setpoint, float process_variable, float dt);
    void reset();

private:
    Settings& settings;
    float integral;
    float previous_error;
    float previous_pv;
    float integralLimit;
};

#endif // PIDCONTROLLER_H
