#include "Settings.h"
#include "Logger.h"
#include <ArduinoJson.h>
#include <LittleFS.h>

// ***************************************************************
// BEGIN: Inicializa LittleFS y carga la configuración
// ***************************************************************
void Settings::begin() {
  bool mounted = LittleFS.begin();

  // Si el montaje falla, intentamos formatear y montar de nuevo.
  // if (!mounted) {
  //     logln("LittleFS mount failed. Checking for corruption and attempting
  //     format..."); if (LittleFS.format()) {
  //         logln("LittleFS successfully formatted. Retrying mount...");
  //         mounted = LittleFS.begin();
  //     } else {
  //         logln("LittleFS format failed.");
  //     }
  // }

  if (!mounted) {
    logln("FATAL: LittleFS still failed to mount after format attempt. "
          "Configuration will use default values only.");
    resetToDefaults();
    return;
  }

  logln("LittleFS mounted successfully.");
  load();
}

// ***************************************************************
// HELPER: Carga los valores por defecto en las variables de la clase
// ***************************************************************
void Settings::resetToDefaults() {
  sensorType = DEFAULT_SENSOR_TYPE;
  sensorWires = DEFAULT_SENSOR_WIRES;
  tempScale = DEFAULT_TEMP_SCALE;
  setpoint = DEFAULT_SETPOINT;
  pidKp = DEFAULT_PID_KP;
  pidTi = DEFAULT_PID_TI;
  pidTd = DEFAULT_PID_TD;
  pidPeriod = DEFAULT_PID_PERIOD;
  stabilityTime = DEFAULT_STABILITY_TIME;
  maxProcessTime = DEFAULT_MAX_PROCESS_TIME;
  calibrationPoints[0] = DEFAULT_P1;
  calibrationPoints[1] = DEFAULT_P2;
  calibrationPoints[2] = DEFAULT_P3;
  alarmUpperLimit = DEFAULT_ALARM_UPPER;
  alarmLowerLimit = DEFAULT_ALARM_LOWER;
  for (int i = 0; i < 3; i++) {
    masterCorrections[i] = DEFAULT_MASTER_CORRECTIONS[i];
    testCorrections[i] = DEFAULT_TEST_CORRECTIONS[i];
  }
  dangerTemperature = DEFAULT_DANGER_TEMP;
  safeTemperature = DEFAULT_SAFE_TEMP;
  minHeatPower = DEFAULT_MIN_HEAT_POWER;
  minCoolPower = DEFAULT_MIN_COOL_POWER;
  maxHeatPower = DEFAULT_MAX_HEAT_POWER;
  maxCoolPower = DEFAULT_MAX_COOL_POWER;
  calibrationTolerance = DEFAULT_CALIBRATION_TOLERANCE;
}

// ***************************************************************
// LOAD: Carga la configuración desde LittleFS (JSON)
// ***************************************************************
void Settings::load() {
  resetToDefaults(); // Primero, establecer los valores por defecto

  // Abre el archivo de configuración en modo lectura
  File configFile = LittleFS.open(CONFIG_FILE, "r");
  if (!configFile) {
    logln("Config: No se encontró el archivo. Usando valores por defecto.");
    save(); // Guardar los valores por defecto recién cargados
    return;
  }

  // Usar StaticJsonDocument para una gestión de memoria más eficiente
  // El tamaño debe ser suficiente (512 bytes)
  StaticJsonDocument<JSON_CONFIG_SIZE> doc;

  // Deserializar el JSON
  DeserializationError error = deserializeJson(doc, configFile);
  configFile.close();

  if (error) {
    logf("Config: Falló la lectura/deserialización del JSON: %s\n",
         error.c_str());
    // El fallo no es crítico, ya cargamos los valores por defecto.
    return;
  }

  logln("Config: JSON cargado y deserializado OK.");

  // Mapeo del JSON a las variables de la clase
  sensorType = (SensorType)doc["sensor_type"].as<int>();
  sensorWires = doc["sensor_wires"].as<int>();
  tempScale = (TemperatureScale)doc["temp_scale"].as<int>();
  setpoint = doc["setpoint"].as<float>();
  pidKp = doc["pid_kp"].as<float>();
  pidTi = doc["pid_ti"].as<float>();
  pidTd = doc["pid_td"].as<float>();
  pidPeriod = doc["pid_period"].as<int>();
  stabilityTime = doc["stability_time"].as<int>();
  maxProcessTime = doc["max_process_time"].as<float>();
  alarmUpperLimit = doc["alarm_upper"].as<float>();
  alarmLowerLimit = doc["alarm_lower"].as<float>();
  dangerTemperature = doc["danger_temp"].as<float>();
  safeTemperature = doc["safe_temp"].as<float>();
  minHeatPower = doc["min_heat_power"].as<float>();
  minCoolPower = doc["min_cool_power"].as<float>();
  maxHeatPower = doc["max_heat_power"].as<float>();
  maxCoolPower = doc["max_cool_power"].as<float>();
  calibrationTolerance = doc["calibration_tolerance"].as<float>();

  // Manejo del Array de Puntos de Calibración
  JsonArray calPoints = doc["cal_points"].as<JsonArray>();
  if (!calPoints.isNull()) {
    int i = 0;
    for (float p : calPoints) {
      if (i < 4) {
        calibrationPoints[i] = p;
        i++;
      }
    }
  }

  // Manejo del Array de Correcciones Master
  JsonArray masterCalArray = doc["master_corrections"].as<JsonArray>();
  if (!masterCalArray.isNull()) {
    int i = 0;
    for (float offset : masterCalArray) {
      if (i < N_POINTS) {
        masterCorrections[i] = offset;
        i++;
      }
    }
  }

  // Manejo del Array de Correcciones Test
  JsonArray testCalArray = doc["test_corrections"].as<JsonArray>();
  if (!testCalArray.isNull()) {
    int i = 0;
    for (float offset : testCalArray) {
      if (i < N_POINTS) {
        testCorrections[i] = offset;
        i++;
      }
    }
  }

  // DEBUG: Verificar qué valores leyó del JSON
  logln("--- DEBUG: Valores Leídos del Archivo ---");
  log("  DEBUG - SETP (File): ");
  logf("%.2f\n", getSetpoint());
  logln("------------------------------------------");
}

// ***************************************************************
// SAVE: Guarda la configuración a LittleFS (JSON)
// ***************************************************************
void Settings::save() {
  StaticJsonDocument<JSON_CONFIG_SIZE> doc;

  // Mapeo de las variables de la clase a JSON
  doc["sensor_type"] = (int)sensorType;
  doc["sensor_wires"] = sensorWires;
  doc["temp_scale"] = (int)tempScale;
  doc["setpoint"] = setpoint;
  doc["pid_kp"] = pidKp;
  doc["pid_ti"] = pidTi;
  doc["pid_td"] = pidTd;
  doc["pid_period"] = pidPeriod;
  doc["stability_time"] = stabilityTime;
  doc["max_process_time"] = maxProcessTime;
  doc["alarm_upper"] = alarmUpperLimit;
  doc["alarm_lower"] = alarmLowerLimit;
  doc["danger_temp"] = dangerTemperature;
  doc["safe_temp"] = safeTemperature;
  doc["min_heat_power"] = minHeatPower;
  doc["min_cool_power"] = minCoolPower;
  doc["max_heat_power"] = maxHeatPower;
  doc["max_cool_power"] = maxCoolPower;
  doc["calibration_tolerance"] = calibrationTolerance;

  // Creación del Array de Puntos de Calibración
  JsonArray calPoints = doc.createNestedArray("cal_points");
  for (int i = 0; i < 3; i++) {
    calPoints.add(calibrationPoints[i]);
  }

  JsonArray masterCalArray = doc.createNestedArray("master_corrections");
  // size_t size = sizeof(masterCorrections) / sizeof(masterCorrections[0]);
  for (int i = 0; i < N_POINTS; i++) {
    masterCalArray.add(masterCorrections[i]);
  }

  JsonArray testCalArray = doc.createNestedArray("test_corrections");
  // size_t size = sizeof(testCorrections) / sizeof(testCorrections[0]);
  for (int i = 0; i < N_POINTS; i++) {
    testCalArray.add(testCorrections[i]);
  }

  // Abre el archivo en modo escritura (crea si no existe, trunca si existe)
  File configFile = LittleFS.open(CONFIG_FILE, "w");
  if (!configFile) {
    logln("Config: Falló la apertura del archivo para escribir.");
    return;
  }

  // Serializa el JSON y lo escribe al archivo
  if (serializeJson(doc, configFile) == 0) {
    logln("Config: Falló la escritura en el archivo.");
  } else {
    logln("Config: Configuración guardada en LittleFS:");
    // Opcional: imprimir el JSON serializado al serial
    // serializeJson(doc, Serial);
  }

  configFile.close();
}

// ============================================================
// HELPERS DE CONVERSIÓN
// ============================================================
float Settings::celsiusTo(float tempC, char targetUnit) const {
  if (targetUnit == 'F' || targetUnit == 'f') {
    return tempC * 1.8f + 32.0f;
  }
  return tempC;
}

float Settings::toCelsius(float temp, char inputUnit) const {
  if (inputUnit == 'F' || inputUnit == 'f') {
    return (temp - 32.0f) / 1.8f;
  }
  return temp;
}

float Settings::deltaToCelsius(float delta, char inputUnit) const {
  if (inputUnit == 'F' || inputUnit == 'f') {
    return delta / 1.8f; // ¡Diferencia, no temperatura!
  }
  return delta;
}

// ============================================================
// SETPOINT
// ============================================================
void Settings::setSetpoint(float temp, char inputUnit) {
  setpoint = toCelsius(temp, inputUnit);
}

float Settings::getSetpoint() const { return setpoint; }

float Settings::getSetpoint(char outputUnit) const {
  return celsiusTo(setpoint, outputUnit);
}

// ============================================================
// PUNTOS DE CALIBRACIÓN
// ============================================================
void Settings::setCalibrationPoint(int index, float temp, char inputUnit) {
  if (index >= 0 && index < 3) {
    calibrationPoints[index] = toCelsius(temp, inputUnit);
  }
}

float Settings::getCalibrationPoint(int index) const {
  if (index >= 0 && index < 3) {
    return calibrationPoints[index];
  }
  return 0.0f;
}

float Settings::getCalibrationPoint(int index, char outputUnit) const {
  return celsiusTo(getCalibrationPoint(index), outputUnit);
}

// ============================================================
// CORRECCIONES
// ============================================================
void Settings::setMasterCorrections(int index, float correction,
                                    char inputUnit) {
  if (index >= 0 && index < 3) {
    masterCorrections[index] = deltaToCelsius(correction, inputUnit);
  }
}

void Settings::setTestCorrections(int index, float correction, char inputUnit) {
  if (index >= 0 && index < 3) {
    testCorrections[index] = deltaToCelsius(correction, inputUnit);
  }
}

float Settings::getMasterCorrections(int index) const {
  if (index >= 0 && index < 3) {
    return masterCorrections[index];
  }
  return 0.0f;
}

float Settings::getTestCorrections(int index) const {
  if (index >= 0 && index < 3) {
    return testCorrections[index];
  }
  return 0.0f;
}

float Settings::getMasterCorrections(int index, char outputUnit) const {
  return celsiusTo(getMasterCorrections(index), outputUnit);
}

float Settings::getTestCorrections(int index, char outputUnit) const {
  return celsiusTo(getTestCorrections(index), outputUnit);
}

// ***************************************************************
// Getters y Setters (Sin cambios, solo se incluyen para la integridad)
// ***************************************************************

SensorType Settings::getSensorType() const { return sensorType; }
void Settings::setSensorType(SensorType type) { sensorType = type; }

int Settings::getSensorWires() const { return sensorWires; }
void Settings::setSensorWires(int wires) { sensorWires = wires; }

TemperatureScale Settings::getTemperatureScale() const { return tempScale; }
void Settings::setTemperatureScale(TemperatureScale scale) {
  tempScale = scale;
}

float Settings::getPidKp() const { return pidKp; }
void Settings::setPidKp(float kp) { pidKp = kp; }

float Settings::getPidTi() const { return pidTi; }
void Settings::setPidTi(float ti) { pidTi = ti; }

float Settings::getPidTd() const { return pidTd; }
void Settings::setPidTd(float td) { pidTd = td; }

float Settings::getPidPeriod() const { return pidPeriod; }
void Settings::setPidPeriod(float period) { pidPeriod = period; }

float Settings::getStabilityTime() const { return stabilityTime; }
void Settings::setStabilityTime(float time) { stabilityTime = time; }

float Settings::getMaxProcessTime() const { return maxProcessTime; }
void Settings::setMaxProcessTime(float time) { maxProcessTime = time; }

// ============================================================
// LÍMITES Y ALARMAS
// ============================================================
void Settings::setAlarmUpperLimit(float temp, char inputUnit) {
  alarmUpperLimit = toCelsius(temp, inputUnit);
}

float Settings::getAlarmUpperLimit() const { return alarmUpperLimit; }

float Settings::getAlarmUpperLimit(char outputUnit) const {
  return celsiusTo(alarmUpperLimit, outputUnit);
}

void Settings::setAlarmLowerLimit(float temp, char inputUnit) {
  alarmLowerLimit = toCelsius(temp, inputUnit);
}

float Settings::getAlarmLowerLimit() const { return alarmLowerLimit; }

float Settings::getAlarmLowerLimit(char outputUnit) const {
  return celsiusTo(alarmLowerLimit, outputUnit);
}

void Settings::setDangerTemperature(float temp, char inputUnit) {
  dangerTemperature = toCelsius(temp, inputUnit);
}

float Settings::getDangerTemperature() const { return dangerTemperature; }

float Settings::getDangerTemperature(char outputUnit) const {
  return celsiusTo(dangerTemperature, outputUnit);
} 

void Settings::setSafeTemperature(float temp, char inputUnit) {
  safeTemperature = toCelsius(temp, inputUnit);
}

float Settings::getSafeTemperature() const { return safeTemperature; }

float Settings::getSafeTemperature(char outputUnit) const {
  return celsiusTo(safeTemperature, outputUnit);
} 

// ============================================================
// CALIBRATION TOLERANCE (es un DELTA)
// ============================================================

// Siempre devuelve en °C (para cálculos internos)
float Settings::getCalibrationTolerance() const { return calibrationTolerance; }

// Devuelve en la unidad solicitada
float Settings::getCalibrationTolerance(char outputUnit) const {
  if (outputUnit == 'F' || outputUnit == 'f') {
    return calibrationTolerance * 1.8f; // Δ°F = Δ°C × 1.8
  }
  return calibrationTolerance;
}

// Establecer tolerancia (inputUnit indica la unidad del valor ingresado)
void Settings::setCalibrationTolerance(float tol, char inputUnit) {
  if (inputUnit == 'F' || inputUnit == 'f') {
    calibrationTolerance = tol / 1.8f; // Convertir delta a °C
  } else {
    calibrationTolerance = tol;
  }
}
