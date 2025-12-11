#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(Settings& settings) : settings(settings) {
    integralLimit = 0.0f;
    reset();
}

void PIDController::reset() {
    integral = 0.0f;
    previous_error = 0.0f;
    previous_pv = 0.0f;
}

float PIDController::calculate(float setpoint, float process_variable, float dt) {
    float kp = settings.getPidKp();
    float ti = settings.getPidTi();
    float td = settings.getPidTd();

    if (dt <= 0.0f) dt = 0.001f; // avoid div by zero

    float error = setpoint - process_variable;

    // Proportional
    float p_out = kp * error;

    // Integral (trapezoidal could be used; simple Euler here) with anti-windup via clamping
    float i_out = 0.0f;
    if (ti > 1e-6f) {
        integral += error * dt;
        // heuristic integral limit to avoid wind-up
        float maxIntegral = (kp > 1e-6f) ? (100.0f * ti / kp) : 1000.0f;
        if (integral > maxIntegral) integral = maxIntegral;
        if (integral < -maxIntegral) integral = -maxIntegral;
        i_out = (kp / ti) * integral;
    } else {
        integral = 0.0f;
    }

    // Derivative on measurement
    float derivative = (process_variable - previous_pv) / dt;
    float d_out = -kp * td * derivative;

    // Total output
    float output = p_out + i_out + d_out;

    // Clamp output to -100..100
    if (output > 100.0f) output = 100.0f;
    else if (output < -100.0f) output = -100.0f;

    previous_error = error;
    previous_pv = process_variable;

    return output;
}
