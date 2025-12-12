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

    unsigned long stabilityStartTime;

private:
    Settings& settings;
    Sensors& sensors;
    Buzzer& buzzer;

    bool running;
    int currentPoint;
    CalibrationData data[8];

    void nextPoint();
    void registerPoint();

    CalibrationRegisteredCallback onPointRegistered = nullptr;
};

#endif // CALIBRATION_H
