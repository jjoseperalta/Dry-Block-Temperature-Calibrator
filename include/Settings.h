#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <LittleFS.h>
// La librería ArduinoJson es necesaria para serializar/deserializar
#include <ArduinoJson.h>

// Tamaño máximo del documento JSON en memoria.
// 512 bytes es un buen valor inicial para esta configuración.
#define JSON_CONFIG_SIZE 1024
#define CONFIG_FILE "/config.json"
#define N_POINTS 3

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

  float getSetpoint() const;
  float getSetpoint(char unit) const;
  void setSetpoint(float temp, char inputUnit = 'C');

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

  float getMaxProcessTime() const;
  void setMaxProcessTime(float time);

  float getCalibrationPoint(int index) const;
  float getCalibrationPoint(int index, char unit) const;
  void setCalibrationPoint(int index, float temp, char unit = 'C');

  float getMasterCorrections(int index) const;
  float getMasterCorrections(int index, char unit) const;
  void setMasterCorrections(int index, float correction, char unit = 'C');

  float getTestCorrections(int index) const;
  float getTestCorrections(int index, char unit) const;
  void setTestCorrections(int index, float correction, char unit = 'C');

  float getAlarmUpperLimit() const;
  float getAlarmUpperLimit(char unit) const;
  void setAlarmUpperLimit(float limit, char unit = 'C');

  float getAlarmLowerLimit() const;
  float getAlarmLowerLimit(char unit) const;
  void setAlarmLowerLimit(float limit, char unit = 'C');

  float getDangerTemperature() const;
  float getDangerTemperature(char unit) const;
  void setDangerTemperature(float temp, char unit = 'C');

  float getSafeTemperature() const;
  float getSafeTemperature(char unit) const;
  void setSafeTemperature(float temp, char unit = 'C');

  float getMinHeatPower() const;
  void setMinHeatPower(float power);

  float getMinCoolPower() const;
  void setMinCoolPower(float power);

  float getMaxHeatPower() const;
  void setMaxHeatPower(float power);

  float getMaxCoolPower() const;
  void setMaxCoolPower(float power);

  float getCalibrationTolerance() const;
  float getCalibrationTolerance(char unit) const;
  void setCalibrationTolerance(float tol, char unit = 'C');

  // Método que permite a otras clases obtener una referencia a la estructura
  // completa (Útil si se usa en la clase Calibrator, como en Calibrador_PT.ino)
  Settings &get() { return *this; }

  static float getDefaultSetpoint() { return DEFAULT_SETPOINT; }
  static float getDefaultPidkp() { return DEFAULT_PID_KP; }
  static float getDefaultPidTi() { return DEFAULT_PID_TI; }
  static float getDefaultPidTd() { return DEFAULT_PID_TD; }
  static float getDefaultPidPeriod() { return DEFAULT_PID_PERIOD; }
  static float getDefaultStabilityTime() { return DEFAULT_STABILITY_TIME; }
  static float getDefaultMaxProcessTime() { return DEFAULT_MAX_PROCESS_TIME; }
  static float getDefaultP1() { return DEFAULT_P1; }
  static float getDefaultP2() { return DEFAULT_P2; }
  static float getDefaultP3() { return DEFAULT_P3; }
  static float getDefaultMasterCorrections(int index) { return DEFAULT_MASTER_CORRECTIONS[index]; }
  static float getDefaultTestCorrections(int index) { return DEFAULT_TEST_CORRECTIONS[index]; }
  static float getDefaultAlarmUpperLimit() { return DEFAULT_ALARM_UPPER; }
  static float getDefaultAlarmLowerLimit() { return DEFAULT_ALARM_LOWER; }
  static float getDefaultDangerTemperature() { return DEFAULT_DANGER_TEMP; }
  static float getDefaultSafeTemperature() { return DEFAULT_SAFE_TEMP; }
  static float getDefaultMinHeatPower() { return DEFAULT_MIN_HEAT_POWER; }
  static float getDefaultMinCoolPower() { return DEFAULT_MIN_COOL_POWER; }
  static float getDefaultMaxHeatPower() { return DEFAULT_MAX_HEAT_POWER; }
  static float getDefaultMaxCoolPower() { return DEFAULT_MAX_COOL_POWER; }
  static float getDefaultCalibrationTolerance() { return DEFAULT_CALIBRATION_TOLERANCE; }

private:
  // *** Eliminamos 'Preferences preferences' ***

  // Variables de estado (sin cambios)
  SensorType sensorType;
  int sensorWires;
  TemperatureScale tempScale;
  float setpoint;
  float pidKp;
  float pidTi;
  float pidTd;
  float pidPeriod;
  float stabilityTime;
  float maxProcessTime;
  float alarmUpperLimit;
  float alarmLowerLimit;
  float calibrationPoints[3];
  float masterCorrections[N_POINTS];
  float testCorrections[N_POINTS];
  float dangerTemperature;
  float safeTemperature;
  float minHeatPower;
  float minCoolPower;
  float maxHeatPower;
  float maxCoolPower;
  float calibrationTolerance;

  // Constantes de valores por defecto (ayudan en load() y resetToDefaults())
  static const SensorType DEFAULT_SENSOR_TYPE = SensorType::PT100;
  static const int DEFAULT_SENSOR_WIRES = 3;
  static const TemperatureScale DEFAULT_TEMP_SCALE = TemperatureScale::CELSIUS;
  static constexpr float DEFAULT_SETPOINT = 20.0f;
  static constexpr float DEFAULT_PID_KP = 1.8f;
  static constexpr float DEFAULT_PID_TI = 0.05f;
  static constexpr float DEFAULT_PID_TD = 1.0f;
  static constexpr float DEFAULT_PID_PERIOD = 1.0f;
  static constexpr float DEFAULT_STABILITY_TIME = 1.0f;
  static constexpr float DEFAULT_MAX_PROCESS_TIME = 800.0f;
  static constexpr float DEFAULT_P1 = 1.111f;
  static constexpr float DEFAULT_P2 = 20.00f;
  static constexpr float DEFAULT_P3 = 38.889f;
  static constexpr float DEFAULT_MASTER_CORRECTIONS[N_POINTS] = {-4.68f, -4.70f, -4.59f};
  static constexpr float DEFAULT_TEST_CORRECTIONS[N_POINTS] = {-5.85f, -7.44f, -8.36f};
  static constexpr float DEFAULT_ALARM_UPPER = 60.0f;
  static constexpr float DEFAULT_ALARM_LOWER = 0.0f;
  static constexpr float DEFAULT_DANGER_TEMP = 40.0f;
  static constexpr float DEFAULT_SAFE_TEMP = 23.0f;
  static constexpr float DEFAULT_MIN_HEAT_POWER = 6.2f;
  static constexpr float DEFAULT_MIN_COOL_POWER = 10.0f;
  static constexpr float DEFAULT_MAX_HEAT_POWER = 90.0f;
  static constexpr float DEFAULT_MAX_COOL_POWER = 90.0f;
  static constexpr float DEFAULT_CALIBRATION_TOLERANCE = 0.167f;

  float celsiusTo(float tempC, char targetUnit) const;
  float toCelsius(float temp, char inputUnit) const;
  float deltaToCelsius(float delta, char unit) const;
};

#endif // SETTINGS_H
