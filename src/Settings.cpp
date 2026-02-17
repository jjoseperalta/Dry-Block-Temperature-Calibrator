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
  setTemperature = DEFAULT_SET_TEMP;
  pidKp = DEFAULT_PID_KP;
  pidTi = DEFAULT_PID_TI;
  pidTd = DEFAULT_PID_TD;
  pidPeriod = DEFAULT_PID_PERIOD;
  stabilityTime = DEFAULT_STABILITY_TIME;
  calibrationPoints[0] = DEFAULT_P1;
  calibrationPoints[1] = DEFAULT_P2;
  calibrationPoints[2] = DEFAULT_P3;
  // calibrationPoints[3] = DEFAULT_P4;
  alarmUpperLimit = DEFAULT_ALARM_UPPER;
  alarmLowerLimit = DEFAULT_ALARM_LOWER;
  masterOffset = DEFAULT_MASTER_OFFSET;
  testOffset = DEFAULT_TEST_OFFSET;
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
  setTemperature = doc["set_temp"].as<float>();
  pidKp = doc["pid_kp"].as<float>();
  pidTi = doc["pid_ti"].as<float>();
  pidTd = doc["pid_td"].as<float>();
  pidPeriod = doc["pid_period"].as<int>();
  stabilityTime = doc["stability_time"].as<int>();
  alarmUpperLimit = doc["alarm_upper"].as<float>();
  alarmLowerLimit = doc["alarm_lower"].as<float>();
  masterOffset = doc["m_cal_off"].as<float>();
  testOffset = doc["t_cal_off"].as<float>();
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

  // DEBUG: Verificar qué valores leyó del JSON
  logln("--- DEBUG: Valores Leídos del Archivo ---");
  log("  DEBUG - SETP (File): ");
  logf("%.2f\n", getSetTemperature());
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
  doc["set_temp"] = setTemperature;
  doc["pid_kp"] = pidKp;
  doc["pid_ti"] = pidTi;
  doc["pid_td"] = pidTd;
  doc["pid_period"] = pidPeriod;
  doc["stability_time"] = stabilityTime;
  doc["alarm_upper"] = alarmUpperLimit;
  doc["alarm_lower"] = alarmLowerLimit;
  doc["m_cal_off"] = masterOffset;
  doc["t_cal_off"] = testOffset;
  doc["danger_temp"] = dangerTemperature;
  doc["safe_temp"] = safeTemperature;
  doc["min_heat_power"] = minHeatPower;
  doc["min_cool_power"] = minCoolPower;
  doc["max_heat_power"] = maxHeatPower;
  doc["max_cool_power"] = maxCoolPower;
  doc["calibration_tolerance"] = calibrationTolerance;

  // Creación del Array de Puntos de Calibración
  JsonArray calPoints = doc.createNestedArray("cal_points");
  for (int i = 0; i < 4; i++) {
    calPoints.add(calibrationPoints[i]);
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
    log("Config: Configuración guardada en LittleFS: ");
    // Opcional: imprimir el JSON serializado al serial
    // serializeJson(doc, Serial);
  }

  configFile.close();
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

float Settings::getSetTemperature() const { return setTemperature; }
void Settings::setSetTemperature(float temp) { setTemperature = temp; }

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

float Settings::getAlarmUpperLimit() const { return alarmUpperLimit; }
void Settings::setAlarmUpperLimit(float limit) { alarmUpperLimit = limit; }

float Settings::getAlarmLowerLimit() const { return alarmLowerLimit; }
void Settings::setAlarmLowerLimit(float limit) { alarmLowerLimit = limit; }

float Settings::getCalibrationPoint(int index) const {
  if (index >= 0 && index < 3) {
    return calibrationPoints[index];
  }
  return 0.0;
}

void Settings::setCalibrationPoint(int index, float temp) {
  if (index >= 0 && index < 3) {
    calibrationPoints[index] = temp;
  }
}

float Settings::getMasterOffset() const { return masterOffset; }

void Settings::setMasterOffset(float offset) { masterOffset = offset; }

float Settings::getTestOffset() const { return testOffset; }

void Settings::setTestOffset(float offset) { testOffset = offset; }

float Settings::getDangerTemperature() const { return dangerTemperature; }

void Settings::setDangerTemperature(float temp) { dangerTemperature = temp; }

float Settings::getSafeTemperature() const { return safeTemperature; }

void Settings::setSafeTemperature(float temp) { safeTemperature = temp; }

// float Settings::getMinHeatPower() const { return minHeatPower; }
// void Settings::setMinHeatPower(float power) {
//   minHeatPower = constrain(power, 0.0f, 100.0f);
// }

// float Settings::getMinCoolPower() const { return minCoolPower; }
// void Settings::setMinCoolPower(float power) {
//   minCoolPower = constrain(power, 0.0f, 100.0f);
// }

// float Settings::getMaxHeatPower() const { return maxHeatPower; }
// void Settings::setMaxHeatPower(float power) {
//   maxHeatPower = constrain(power, 0.0f, 100.0f);
// }

// float Settings::getMaxCoolPower() const { return maxCoolPower; }
// void Settings::setMaxCoolPower(float power) {
//   maxCoolPower = constrain(power, 0.0f, 100.0f);
// }

float Settings::getCalibrationTolerance() const { return calibrationTolerance; }
void Settings::setCalibrationTolerance(float tol) {
  calibrationTolerance = constrain(tol, 0.02f, 0.5f);
}