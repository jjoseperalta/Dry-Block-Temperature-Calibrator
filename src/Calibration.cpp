#include "Calibration.h"
#include "Logger.h"
#include <Arduino.h>

Calibration::Calibration(Settings &settings, Buzzer &buzzer)
    : settings(settings), buzzer(buzzer) {}

void Calibration::start() {
  if (running) {
    logln("Calibration is already running.");
    return;
  }
  logln("Starting calibration process...");
  setupPoints();
  running = true;
  currentPoint = 0;
  stepStartTime = millis();
  stabilityStartTime = 0;

  settings.setSetpoint(data[currentPoint].setpoint);

  // buzzer.playBlocking(BeepType::START_PROCESS);

  logln("====================================");
  logln("STARTING AUTO CALIBRATION");
  logln("====================================");
  logf("TOLERANCE: ±%.3f°C / ±%.3f°F\n", settings.getCalibrationTolerance('C'), settings.getCalibrationTolerance('F'));
  logf(">>> Point %d/%d: %.2f C (%s)\n", currentPoint + 1, TOTAL_POINTS,
       data[currentPoint].setpoint,
       data[currentPoint].isRising ? "HEATING" : "COOLING");
}

void Calibration::stop() {
  if (!running) {
    logln("Calibration is not running.");
    return;
  }

  running = false;
  // currentPoint = 0;
  stabilityStartTime = 0;

  buzzer.playBlocking(BeepType::STOP);

  logln("Calibration stopped.");
}

void Calibration::loop(float masterTemp, float testTemp) {
  if (!running)
    return;

  CalibrationData &point = data[currentPoint];

  const float tolerance = settings.getCalibrationTolerance();

  const float error = fabsf(point.setpoint - masterTemp);

  const bool stable = error <= tolerance;

  // =========================================
  // STABLE ZONE
  // =========================================

  if (stable) {
    if (stabilityStartTime == 0) {
      stabilityStartTime = millis();

      lastStableTemp = masterTemp;

      logf("[%.2f C STABLE] Holding for %.0f seconds...\n", masterTemp,
           settings.getStabilityTime());
    } else {
      stableElapsed = millis() - stabilityStartTime;

      if (stableElapsed >= (settings.getStabilityTime() * 1000.0f)) {
        registerPoint(masterTemp, testTemp);

        nextPoint();
      }
    }
  } else {
    stabilityStartTime = 0;
    stableElapsed = 0;
  }

  // =========================================
  // TIMEOUT
  // =========================================

  uint32_t elapsed = millis() - stepStartTime;

  if (elapsed >= settings.getMaxProcessTime() * 1000.0f) {
    // logln("TIMEOUT: Setpoint not reached.");

    CalibrationData &timeoutPoint = data[currentPoint];

    timeoutPoint.masterTemp = masterTemp;
    timeoutPoint.testTemp = testTemp;

    timeoutPoint.difference = testTemp - masterTemp;
  }
}

bool Calibration::isRunning() const { return running; }

float Calibration::getCurrentSetpoint() const {
  return data[currentPoint].setpoint;
}

void Calibration::setupPoints() {
  data[0].setpoint = settings.getCalibrationPoint(0);
  data[1].setpoint = settings.getCalibrationPoint(1);
  data[2].setpoint = settings.getCalibrationPoint(2);
  data[3].setpoint = settings.getCalibrationPoint(1);
  data[4].setpoint = settings.getCalibrationPoint(0);

  for (int i = 0; i < TOTAL_POINTS; i++) {
    data[i].reached = false;

    if (i == 0) {
      data[i].isRising = false;
    } else {
      data[i].isRising = data[i].setpoint > data[i - 1].setpoint;
    }
  }
}

void Calibration::nextPoint() {
  currentPoint++;

  if (currentPoint >= TOTAL_POINTS) {
    running = false;
    buzzer.playBlocking(BeepType::STOP);
    logln("Calibration completed.");
    printReport();
    return;
  }

  settings.setSetpoint(data[currentPoint].setpoint);

  stepStartTime = millis();
  stabilityStartTime = 0;

  logf(">>> Point %d/%d: %.2f C (%s)\n", currentPoint + 1, TOTAL_POINTS,
       data[currentPoint].setpoint,
       data[currentPoint].isRising ? "HEATING" : "COOLING");
}

void Calibration::registerPoint(float masterTemp, float testTemp) {
  CalibrationData &point = data[currentPoint];

  point.masterTemp = masterTemp;
  point.testTemp = testTemp;

  point.difference = testTemp - masterTemp;

  point.timeToReach = (millis() - stepStartTime) / 1000;

  point.reached = true;

  buzzer.playBlocking(BeepType::POINT_REGISTERED);

  logf("State: CAL POINT REACHED | "
       "Setpoint: %.2f | "
       "Master: %.2f | "
       "Test: %.2f | "
       "Diff: %.3f | "
       "Time: %s | "
       "Dir: %s\n",
       point.setpoint, point.masterTemp, point.testTemp, point.difference,
       formatTimeHMS(point.timeToReach).c_str(),
       point.isRising ? "UP" : "DOWN");

  if (onPointRegistered) {
    onPointRegistered(currentPoint);
  }
}

void Calibration::setRegisterCallback(CalibrationRegisteredCallback callback) {
  onPointRegistered = callback;
}

const CalibrationData &Calibration::getCalibrationData(int index) const {
  if (index < 0 || index >= TOTAL_POINTS) {
    return data[0];
  }

  return data[index];
}

void Calibration::printReport() {
  logln("\n===============================================================================");
  logln("                         CALIBRATION REPORT");
  logln("===============================================================================");
  
  // Cabecera con ambas escalas siempre visibles
  logln("+----+---------------+---------------+---------------+---------------+----------+------+");
  logln("| #  |      SETPOINT |     REFERENCE |    DEVICE UT  |         DIFF  |   TIME   | DIR  |");
  logln("|    |   C       [F] |   C       [F] |   C       [F] |   C       [F] |  (HH:MM) |      |");
  logln("+----+---------------+---------------+---------------+---------------+----------+------+");

  float avgError = 0.0f;
  float avgDiff = 0.0f;
  int successCount = 0;
  unsigned long totalTimeSeconds = 0;

  for (int i = 0; i < TOTAL_POINTS; i++) {
    const CalibrationData &p = data[i];
    
    // Convertir a Fahrenheit
    float spF = p.setpoint * 1.8f + 32.0f;
    float masterF = p.masterTemp * 1.8f + 32.0f;
    float testF = p.testTemp * 1.8f + 32.0f;
    float diffF = p.difference * 1.8f;
    
    // Formato: siempre ambas unidades
    logf("| %2d | %6.2f %6.2f | %6.2f %6.2f | %6.2f %6.2f | %+6.3f %+6.3f | %8s | %4s |\n",
         i + 1,
         p.setpoint, spF,
         p.masterTemp, masterF,
         p.testTemp, testF,
         p.difference, diffF,
         formatTimeHMS(p.timeToReach).c_str(),
         p.isRising ? "UP" : "DOWN");

    if (p.reached) {
      avgError += fabsf(p.masterTemp - p.setpoint);
      avgDiff += fabsf(p.testTemp - p.masterTemp);
      successCount++;
      totalTimeSeconds += p.timeToReach;
    }
  }

  avgError = successCount > 0 ? avgError / successCount : 0.0f;
  avgDiff = successCount > 0 ? avgDiff / successCount : 0.0f;
  float avgTime = successCount > 0 ? (float)totalTimeSeconds / successCount : 0.0f;
  
  float avgErrorF = avgError * 1.8f;
  float avgDiffF = avgDiff * 1.8f;

  logln("+---+---------------------+---------------------+---------------------+---------------------+----------+------+--------+");
  logln("|                             SUMMARY STATISTICS                                                             |");
  logln("+---------------------------------------------------------------------------------------------+---------------+");
  logf  ("| Successful points:                             %2d / %2d                                                    |\n", successCount, TOTAL_POINTS);
  logf  ("| Avg Error (Master vs Setpoint):               %6.3f C  [%6.3f F]                                       |\n", avgError, avgErrorF);
  logf  ("| Avg Difference (Test vs Master):              %6.3f C  [%6.3f F]                                       |\n", avgDiff, avgDiffF);
  // logf  ("| Avg Time to Setpoint:                          %s                                                       |\n", formatTimeHMS(avgTime).c_str());
  
  // Veredicto
  logln("+---------------------------------------------------------------------------------------------+---------------+");
  // if (avgDiff < 0.2f) {
  //   logln("| VERDICT: Sensors are WELL MATCHED (Delta < 0.2 C)                                              |");
  // } else if (avgDiff < 0.5f) {
  //   logln("| VERDICT: Sensors are ACCEPTABLE (Delta < 0.5 C) - Consider recalibration                        |");
  // } else {
  //   logln("| VERDICT: Sensors MISMATCHED (Delta >= 0.5 C) - Calibration recommended                          |");
  // }
  // logln("===============================================================================\n");
}

String Calibration::formatTimeHMS(uint32_t seconds) {
  uint32_t hours = seconds / 3600;
  uint32_t minutes = (seconds % 3600) / 60;
  uint32_t secs = seconds % 60;

  char buffer[9]; // HH:MM:SS son 8 chars + null terminator
  snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u", hours, minutes, secs);
  return String(buffer);
}

void Calibration::runDemo() {
  setupPoints();
  if (running) {
        logln("⚠ Calibration is already running. Stop it first.");
        return;
    }

    for (int i = 0; i < TOTAL_POINTS; i++) {
    currentPoint = i; // Movemos el índice nativo de la clase

    // Simular un tiempo de estabilización ficticio (ej: 45 segundos fijos)
    stepStartTime = millis() - (45 * 1000);

    // --- VARIACIÓN ALEATORIA FLOTANTE DE ±0.10°C ---
    float r1 = ((float)random(0, 10000) / 10000.0f) * 2.0f - 1.0f; // Escala entre -1.0 y +1.0
    float r2 = ((float)random(0, 10000) / 10000.0f) * 2.0f - 1.0f;

    // Tomamos el setpoint del slot actual que inicializó setupPoints()
    float baseSetpoint = data[currentPoint].setpoint;

    float masterSimulado = baseSetpoint + (r1 * 0.10f);
    float testSimulado   = baseSetpoint + (r2 * 0.10f);

    // 3. Invocar tu método de registro original.
    // Esto calculará la diferencia, pondrá reached = true, sonará el buzzer,
    // imprimirá el log individual y disparará el callback 'onPointRegistered' a main.cpp.
    registerPoint(masterSimulado, testSimulado);
    nextPoint();
    // Pausa técnica para dejar respirar los buffers serie de la HMI
    delay(150);
  }
}