#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "Settings.h"
#include "Sensors.h"
#include "Buzzer.h"

struct CalibrationData {
    float setpoint;
    float masterTemp;
    float testTemp;
    float difference;
};

typedef void (*CalibrationRegisteredCallback)(int pointIndex);

class Calibration {
public:
    Calibration(Settings& settings, Sensors& sensors, Buzzer& buzzer);
    void start();
    void stop();
    void loop();
    bool isRunning() const;
    void targetReached();
    // const CalibrationData& getCalibrationData() const;
    const CalibrationData& getCalibrationData(int index) const;
    void setRegisterCallback(CalibrationRegisteredCallback callback);
    void notifyStable(float masterTemp, float testTemp);

    unsigned long stabilityStartTime;

private:
    Settings& settings;
    Sensors& sensors;
    Buzzer& buzzer;

    bool running;
    uint16_t currentPoint;
    uint16_t stableCounter;
    const uint16_t requiredStableSamples = 0;
    CalibrationData data[6];

    void nextPoint();
    void registerPoint(float masterTemp, float testTemp);
    void tryRegisterPoint();

    CalibrationRegisteredCallback onPointRegistered = nullptr;
};

#endif // CALIBRATION_H
