#include "Calibration.h"
#include "Logger.h"
#include <Arduino.h>

Calibration::Calibration(Settings &settings, Sensors &sensors, Buzzer &buzzer)
    : settings(settings), sensors(sensors), buzzer(buzzer), running(false),
      currentPoint(0), stabilityStartTime(0) {}

void Calibration::start() {
  if (running) {
    logln("Calibration is already running.");
    return;
  }
  logln("Starting calibration process...");
  running = true;
  currentPoint = 0;
  stabilityStartTime = 0;
  stableCounter = 0;
  nextPoint();
}

void Calibration::stop() {
  if (!running) {
    logln("Calibration is not running.");
    return;
  }

  running = false;
  currentPoint = 0;
  stabilityStartTime = 0;
  stableCounter = 0;

  logln("Calibration stopped.");
}

void Calibration::loop() {
  // if (!running) {
  //   return;
  // }

  // float master = sensors.readMasterTemperature();
  // static float lastMaster = master;

  // if (fabs(master - lastMaster) < 0.02f) {
  //   registerPoint();
  //   nextPoint();
  // }

  // lastMaster = master;
}

bool Calibration::isRunning() const { return running; }

void Calibration::targetReached() {
  if (isRunning()) {
    stabilityStartTime = millis();
    logln("Target temperature reached. Stabilizing...");
  }
}

void Calibration::nextPoint() {
  if (currentPoint >= 8) {
    stop();
    logln("Calibration finished.");
    // Print results
    logln("Calibration Results:");
    logln("Setpoint | Master Temp | Test Temp | Difference (%)");
    for (int i = 0; i < 8; i++) {
      logf("%.2f | %.2f | %.2f | %.2f\n", data[i].setpoint, data[i].masterTemp,
           data[i].testTemp, data[i].difference);
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
  logf("Moving to calibration point %d: %.2f C\n", currentPoint + 1, setpoint);
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
  logf("Point %d registered: Master=%.2f, Test=%.2f, Diff=%.2f%%\n", index + 1,
       masterTemp, testTemp, diff);

  currentPoint++;
  stabilityStartTime = 0;

  // int newPointIndex = 0;
  if (onPointRegistered) {
    onPointRegistered(index);
  }
}

void Calibration::setRegisterCallback(CalibrationRegisteredCallback callback) {
  onPointRegistered = callback;
}

const CalibrationData &Calibration::getCalibrationData(int index) const {
  // Comprobación de rango básico. Si el índice está fuera de rango,
  // es mejor devolver un valor seguro o, en este caso, el primer elemento
  // (índice 0) para evitar un fallo de memoria fuera de límites.
  if (index >= 0 && index < 8) {
    return data[index];
  }

  // Si el índice es inválido, devuelve el primer elemento (o lanza una
  // excepción si usaras STL) Es una solución segura para evitar crashes en
  // sistemas embebidos.
  return data[0];
}

void Calibration::notifyStable() {
  if (!running)
    return;

  stableCounter++;

  if (stableCounter >= requiredStableSamples) {
    tryRegisterPoint();
    stableCounter = 0;
  }
}

void Calibration::tryRegisterPoint() {

  float tol = settings.getCalibrationTolerance();
  float masterTemp = sensors.readMasterTemperature();
  float testTemp = sensors.readTestTemperature();

  // Protección SOLO lógica (no física)
  if (fabs(masterTemp - testTemp) > tol) {
    logf("Calibration rejected: |%.3f - %.3f| > %.3f\n", masterTemp, testTemp,
         tol);
    return;
  }

  registerPoint();
  nextPoint();
}
