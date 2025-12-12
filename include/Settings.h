#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <LittleFS.h>
// La librería ArduinoJson es necesaria para serializar/deserializar
#include <ArduinoJson.h> 

// Tamaño máximo del documento JSON en memoria. 
// 512 bytes es un buen valor inicial para esta configuración.
#define JSON_CONFIG_SIZE 512
#define CONFIG_FILE "/config.json"

enum class SensorType { PT100, PT1000 };
enum class TemperatureScale { CELSIUS, FAHRENHEIT };

class Settings {
public:
    // La función begin() ya no necesita inicializar LittleFS aquí, 
    // se asume que ya fue montado en el .ino principal.
    void begin();
    void load();
    void save();
    void resetToDefaults();

    // Métodos Getter/Setter... (sin cambios)
    SensorType getSensorType() const;
    void setSensorType(SensorType type);

    int getSensorWires() const;
    void setSensorWires(int wires);

    TemperatureScale getTemperatureScale() const;
    void setTemperatureScale(TemperatureScale scale);

    float getSetTemperature() const;
    void setSetTemperature(float temp);

    float getPidKp() const;
    void setPidKp(float kp);

    float getPidTi() const;
    void setPidTi(float ti);

    float getPidTd() const;
    void setPidTd(float td);

    float getPidPeriod() const;
    void setPidPeriod(float period);

    float getStabilityTime() const;
    void setStabilityTime(float time);

    float getMasterOffset() const;
    void setMasterOffset(float offset);

    float getTestOffset() const;
    void setTestOffset(float offset);

    float getAlarmUpperLimit() const;
    void setAlarmUpperLimit(float limit);

    float getAlarmLowerLimit() const;
    void setAlarmLowerLimit(float limit);

    float getCalibrationPoint(int index) const;
    void setCalibrationPoint(int index, float temp);

    float getDangerTemperature() const;
    void setDangerTemperature(float temp);

    float getSafeTemperature() const;
    void setSafeTemperature(float temp);

    // Método que permite a otras clases obtener una referencia a la estructura completa
    // (Útil si se usa en la clase Calibrator, como en Calibrador_PT.ino)
    Settings& get() { return *this; }

    static float getDefaultPidkp() { return DEFAULT_PID_KP; }
    static float getDefaultPidTi() { return DEFAULT_PID_TI; }
    static float getDefaultPidTd() { return DEFAULT_PID_TD; }
    static float getDefaultPidPeriod() { return DEFAULT_PID_PERIOD; }
    static float getDefaultStabilityTime() { return DEFAULT_STABILITY_TIME; }
    static float getDefaultP1() { return DEFAULT_P1; }
    static float getDefaultP2() { return DEFAULT_P2; }
    static float getDefaultP3() { return DEFAULT_P3; }
    static float getDefaultP4() { return DEFAULT_P4; }
    static float getDefaultMasterOffset() { return DEFAULT_MASTER_OFFSET; }
    static float getDefaultTestOffset() { return DEFAULT_TEST_OFFSET; }
    static float getDefaultAlarmUpperLimit() { return DEFAULT_ALARM_UPPER; }
    static float getDefaultAlarmLowerLimit() { return DEFAULT_ALARM_LOWER; }
    static float getDefaultDangerTemperature() { return DEFAULT_DANGER_TEMP; }
    static float getDefaultSafeTemperature() { return DEFAULT_SAFE_TEMP; }

private:
    // *** Eliminamos 'Preferences preferences' ***

    // Variables de estado (sin cambios)
    SensorType sensorType;
    int sensorWires;
    TemperatureScale tempScale;
    float setTemperature;
    float pidKp;
    float pidTi;
    float pidTd;
    float pidPeriod;
    float stabilityTime;
    float alarmUpperLimit;
    float alarmLowerLimit;
    float calibrationPoints[4]; // Array para 4 puntos de calibración
    float masterOffset;
    float testOffset;
    float dangerTemperature;
    float safeTemperature;

    // Constantes de valores por defecto (ayudan en load() y resetToDefaults())
    static const SensorType DEFAULT_SENSOR_TYPE = SensorType::PT100;
    static const int DEFAULT_SENSOR_WIRES = 2;
    static const TemperatureScale DEFAULT_TEMP_SCALE = TemperatureScale::CELSIUS;
    static constexpr float DEFAULT_SET_TEMP = 25.0;
    static constexpr float DEFAULT_PID_KP = 8.0;
    static constexpr float DEFAULT_PID_TI = 700; //300
    static constexpr float DEFAULT_PID_TD = 8.0;
    static constexpr float DEFAULT_PID_PERIOD = 1;
    static constexpr float DEFAULT_STABILITY_TIME = 60.0;
    static constexpr float DEFAULT_P1 = 30.0;
    static constexpr float DEFAULT_P2 = 35.0;
    static constexpr float DEFAULT_P3 = 40.0;
    static constexpr float DEFAULT_P4 = 45.0;
    static constexpr float DEFAULT_MASTER_OFFSET = 0.0;
    static constexpr float DEFAULT_TEST_OFFSET = 0.0;
    static constexpr float DEFAULT_ALARM_UPPER = 100.0;
    static constexpr float DEFAULT_ALARM_LOWER = 10.0;
    static constexpr float DEFAULT_DANGER_TEMP = 40.0;
    static constexpr float DEFAULT_SAFE_TEMP = 35.0;
};

#endif // SETTINGS_H