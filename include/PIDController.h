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
  PIDController(Settings &settings);
  float calculate(float setpoint, float process_variable, float dt);
  void reset();
  void setPreviousPV(float pv) { previous_pv = pv; }
  bool isInFineZone() const { return inFineZone; }
  float compute(float input, float setpoint,
                float dt);
  void setIntegral(float value);

private:
  Settings &settings;

  float integral = 0.0f;
  float previous_pv = 0.0f;
  bool inFineZone = false;
  float lastError = 0.0f;

  PIDGains computeGains(float error, bool fineZone);
};

#endif // PIDCONTROLLER_H
