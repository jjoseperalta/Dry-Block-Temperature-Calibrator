#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Settings.h"

struct PIDGains {
    float kp;
    float ti;
    float td;
};

class PIDController {
public:
    PIDController(Settings& settings);
    // dt: elapsed time in seconds since last calculation
    float calculate(float setpoint, float process_variable, float dt);
    void reset();
    void setPreviousPV(float pv) { previous_pv = pv; }
    bool isInFineZone() const { return inFineZone; }

private:
    Settings& settings;

    float integral = 0.0f;
    float previous_pv = 0.0f;
    bool inFineZone = false;
    bool lastHeating = true;


    // static constexpr float MIN_HEAT = 6.0f;   // %
    // static constexpr float MIN_COOL = 10.0f;  // %
    
    PIDGains computeGains(float error, bool fineZone);
};

#endif // PIDCONTROLLER_H
