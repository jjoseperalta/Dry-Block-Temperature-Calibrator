#include "Settings.h"

// ***************************************************************
// BEGIN: Inicializa LittleFS y carga la configuración
// ***************************************************************
void Settings::begin() {
    bool mounted = LittleFS.begin();

    // Si el montaje falla, intentamos formatear y montar de nuevo.
    // if (!mounted) {
    //     Serial.println("LittleFS mount failed. Checking for corruption and attempting format...");
    //     if (LittleFS.format()) {
    //         Serial.println("LittleFS successfully formatted. Retrying mount...");
    //         mounted = LittleFS.begin();
    //     } else {
    //         Serial.println("LittleFS format failed.");
    //     }
    // }
    
    if (!mounted) {
        Serial.println("FATAL: LittleFS still failed to mount after format attempt. Configuration will use default values only.");
        resetToDefaults();
        return;
    }
    
    Serial.println("LittleFS mounted successfully.");
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
    calibrationPoints[3] = DEFAULT_P4;
    alarmUpperLimit = DEFAULT_ALARM_UPPER;
    alarmLowerLimit = DEFAULT_ALARM_LOWER;
    masterOffset = DEFAULT_MASTER_OFFSET;
    testOffset = DEFAULT_TEST_OFFSET;
    dangerTemperature = DEFAULT_DANGER_TEMP;
    safeTemperature = DEFAULT_SAFE_TEMP;
}

// ***************************************************************
// LOAD: Carga la configuración desde LittleFS (JSON)
// ***************************************************************
void Settings::load() {
    resetToDefaults(); // Primero, establecer los valores por defecto
    
    // Abre el archivo de configuración en modo lectura
    File configFile = LittleFS.open(CONFIG_FILE, "r");
    if (!configFile) {
        Serial.println("Config: No se encontró el archivo. Usando valores por defecto.");
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
        Serial.print(F("Config: Falló la lectura/deserialización del JSON: "));
        Serial.println(error.f_str());
        // El fallo no es crítico, ya cargamos los valores por defecto.
        return;
    }

    Serial.println("Config: JSON cargado y deserializado OK.");

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
    Serial.println(F("--- DEBUG: Valores Leídos del Archivo ---"));
    Serial.print(F("  DEBUG - SETP (File): ")); Serial.println(setTemperature, 2);
    Serial.println(F("------------------------------------------"));
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

    // Creación del Array de Puntos de Calibración
    JsonArray calPoints = doc.createNestedArray("cal_points");
    for (int i = 0; i < 4; i++) {
        calPoints.add(calibrationPoints[i]);
    }

    // Abre el archivo en modo escritura (crea si no existe, trunca si existe)
    File configFile = LittleFS.open(CONFIG_FILE, "w");
    if (!configFile) {
        Serial.println("Config: Falló la apertura del archivo para escribir.");
        return;
    }

    // Serializa el JSON y lo escribe al archivo
    if (serializeJson(doc, configFile) == 0) {
        Serial.println(F("Config: Falló la escritura en el archivo."));
    } else {
        Serial.print("Config: Configuración guardada en LittleFS: ");
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
void Settings::setTemperatureScale(TemperatureScale scale) { tempScale = scale; }

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
    if (index >= 0 && index < 4) {
        return calibrationPoints[index];
    }
    return 0.0;
}

void Settings::setCalibrationPoint(int index, float temp) {
    if (index >= 0 && index < 4) {
        calibrationPoints[index] = temp;
    }
}

float Settings::getMasterOffset() const {
    return masterOffset;
}

void Settings::setMasterOffset(float offset) {
    masterOffset = offset;
}

float Settings::getTestOffset() const {
    return testOffset;
}

void Settings::setTestOffset(float offset) {
    testOffset = offset;
}

float Settings::getDangerTemperature() const {
    return dangerTemperature;
}

void Settings::setDangerTemperature(float temp) {
    dangerTemperature = temp;
}

float Settings::getSafeTemperature() const {
    return safeTemperature;
}

void Settings::setSafeTemperature(float temp) {
    safeTemperature = temp;
}