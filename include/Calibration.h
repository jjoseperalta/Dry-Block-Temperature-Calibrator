#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "Settings.h"
#include "Sensors.h"
#include "Buzzer.h"

struct CalibrationData {
    float setpoint = 0.0f;
    float masterTemp = 0.0f;
    float testTemp = 0.0f;
    float difference = 0.0f;
    uint32_t timeToReach = 0;
    bool isRising = false;
    bool reached = false;
};

typedef void (*CalibrationRegisteredCallback)(int pointIndex);

class Calibration {
public:
    static constexpr uint8_t TOTAL_POINTS = 5;
    
    Calibration(Settings& settings, Buzzer& buzzer);

    void start();
    void stop();

    void loop(float masterTemp, float testTemp);

    bool isRunning() const;

    float getCurrentSetpoint() const;

    const CalibrationData& getCalibrationData(int index) const;
    void setRegisterCallback(CalibrationRegisteredCallback callback);

    String formatTimeHMS(uint32_t seconds);

    // int getStabilityStartTime() const { return stabilityStartTime; }

    uint32_t stableElapsed = 0;

    uint32_t getStableElapsed() const { return stableElapsed; }

    bool isBrakingStep() const { return running && currentPoint == 3; }

    void runDemo();

private:
    Settings& settings;
    Buzzer& buzzer;

    bool running = false;
    
    CalibrationData data[TOTAL_POINTS];

    uint8_t currentPoint = 0;

    uint32_t stepStartTime = 0;
    uint32_t stabilityStartTime = 0;

    float lastStableTemp = 0.0f;

    void setupPoints();
    void nextPoint();
    void registerPoint(float masterTemp, float testTemp);
    void printReport();

    CalibrationRegisteredCallback onPointRegistered = nullptr;
};

#endif // CALIBRATION_H
