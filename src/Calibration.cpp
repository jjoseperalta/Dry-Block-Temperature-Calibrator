#include "Calibration.h"
#include <Arduino.h>

Calibration::Calibration(Settings& settings, Sensors& sensors, Buzzer& buzzer) :
    settings(settings), sensors(sensors), buzzer(buzzer), running(false), currentPoint(0), stabilityStartTime(0) {}

void Calibration::start() {
    if (running) {
        Serial.println("Calibration is already running.");
        return;
    }
    Serial.println("Starting calibration process...");
    running = true;
    currentPoint = 0;
    stabilityStartTime = 0;
    nextPoint();
}

void Calibration::stop() {
    if (!running) {
        Serial.println("Calibration is not running.");
        return;
    }
    running = false;
    Serial.println("Calibration stopped.");
}

void Calibration::loop() {
    if (!running) {
        return;
    }

    if (stabilityStartTime > 0 && millis() - stabilityStartTime >= settings.getStabilityTime() * 1000) {
        registerPoint();
        nextPoint();
    }
}

bool Calibration::isRunning() const {
    return running;
}

void Calibration::targetReached() {
    if (isRunning()) {
        stabilityStartTime = millis();
        Serial.println("Target temperature reached. Stabilizing...");
    }
}

void Calibration::nextPoint() {
    if (currentPoint >= 8) {
        stop();
        Serial.println("Calibration finished.");
        // Print results
        Serial.println("Calibration Results:");
        Serial.println("Setpoint | Master Temp | Test Temp | Difference (%)");
        for (int i = 0; i < 8; i++) {
            Serial.printf("%.2f | %.2f | %.2f | %.2f\n",
                data[i].setpoint, data[i].masterTemp, data[i].testTemp, data[i].difference);
        }
        return;
    }

    float setpoint;
    if (currentPoint < 4) {
        setpoint = settings.getCalibrationPoint(currentPoint);
    } else {
        setpoint = settings.getCalibrationPoint(7 - currentPoint);
    }

    settings.setSetTemperature(setpoint);
    Serial.printf("Moving to calibration point %d: %.2f C\n", currentPoint + 1, setpoint);
}

void Calibration::registerPoint() {
    float masterTemp = sensors.readMasterTemperature();
    float testTemp = sensors.readTestTemperature();
    float diff = 0.0;
    if (masterTemp != 0) {
        diff = ((testTemp - masterTemp) / masterTemp) * 100.0;
    }

    int index = currentPoint;

    data[index].setpoint = settings.getSetTemperature();
    data[index].masterTemp = masterTemp;
    data[index].testTemp = testTemp;
    data[index].difference = diff;

    buzzer.beep(BeepType::POINT_REGISTERED);
    Serial.printf("Point %d registered: Master=%.2f, Test=%.2f, Diff=%.2f%%\n",
        index + 1, masterTemp, testTemp, diff);

    currentPoint++;
    stabilityStartTime = 0;
}
