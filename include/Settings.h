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

    int getPidPeriod() const;
    void setPidPeriod(int period);

    int getStabilityTime() const;
    void setStabilityTime(int time);

    float getAlarmUpperLimit() const;
    void setAlarmUpperLimit(float limit);

    float getAlarmLowerLimit() const;
    void setAlarmLowerLimit(float limit);

    float getCalibrationPoint(int index) const;
    void setCalibrationPoint(int index, float temp);

    float getMasterCalibrationOffset() const;
    void setMasterCalibrationOffset(float offset);

    // Método que permite a otras clases obtener una referencia a la estructura completa
    // (Útil si se usa en la clase Calibrator, como en Calibrador_PT.ino)
    Settings& get() { return *this; }

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
    int pidPeriod;
    int stabilityTime;
    float alarmUpperLimit;
    float alarmLowerLimit;
    float calibrationPoints[4]; // Array para 4 puntos de calibración
    float masterCalibrationOffset;

    // Constantes de valores por defecto (ayudan en load() y resetToDefaults())
    static const SensorType DEFAULT_SENSOR_TYPE = SensorType::PT100;
    static const int DEFAULT_SENSOR_WIRES = 3;
    static const TemperatureScale DEFAULT_TEMP_SCALE = TemperatureScale::CELSIUS;
    static constexpr float DEFAULT_SET_TEMP = 25.0;
    static constexpr float DEFAULT_PID_KP = 1.0;
    static constexpr float DEFAULT_PID_TI = 10.0;
    static constexpr float DEFAULT_PID_TD = 0.1;
    static const int DEFAULT_PID_PERIOD = 1;
    static const int DEFAULT_STABILITY_TIME = 1;
    static constexpr float DEFAULT_ALARM_UPPER = 150.0;
    static constexpr float DEFAULT_ALARM_LOWER = -10.0;
    static constexpr float DEFAULT_CAL_OFFSET = 0.0;
};

#endif // SETTINGS_H