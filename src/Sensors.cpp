#include "Sensors.h"
#include "Logger.h"
#include <Arduino.h>

Sensors::Sensors(Settings &settings)
    : settings(settings), masterSensor(Adafruit_MAX31865(
                              MAX_CS_MASTER, MAX_MOSI, MAX_MISO, MAX_SCK)),
      testSensor(Adafruit_MAX31865(MAX_CS_TEST, MAX_MOSI, MAX_MISO, MAX_SCK)) {}

void Sensors::begin() {
  masterSensor.begin(MAX31865_3WIRE);
  logf("Master Sensor Initialized (CS: %d)\n", MAX_CS_MASTER);

  configureTestSensor(settings.getSensorType(), settings.getSensorWires());
  logf("Test Sensor Initialized (CS: %d)\n", MAX_CS_TEST);
}

// ***************************************************************
// HELPER: Verifica y loguea los fallos del MAX31865
// ***************************************************************
bool Sensors::checkAndLogFault(Adafruit_MAX31865 &sensor,
                               const String &sensorName) {
  uint8_t fault = sensor.readFault();
  if (fault) {
    logf("ERROR: %s Fault Detected (0x%X): ", sensorName, fault);

    if (fault & MAX31865_FAULT_HIGHTHRESH)
      log("High Threshold, ");
    if (fault & MAX31865_FAULT_LOWTHRESH)
      log("Low Threshold, ");
    if (fault & MAX31865_FAULT_REFINLOW)
      log("REFIN Low, ");
    if (fault & MAX31865_FAULT_REFINHIGH)
      log("REFIN High, ");
    if (fault & MAX31865_FAULT_RTDINLOW)
      log("RTD Low, ");
    if (fault & MAX31865_FAULT_OVUV)
      log("Over/Under Voltage");
    logln("");
    return true; // Hay un fallo
  }
  return false; // No hay fallo
}

// ***************************************************************
// HELPER: Aplica la conversión de Celsius a Fahrenheit si es necesario
// ***************************************************************
float Sensors::applyScaleConversion(float tempC) {
  if (settings.getTemperatureScale() == TemperatureScale::FAHRENHEIT) {
    return tempC * 1.8 + 32.0f;
  }
  return tempC;
}

// ***************************************************************
// LECTURA: Sensor Maestro
// ***************************************************************
float Sensors::readMasterTemperature() {
  // 1. Lectura de temperatura
  float tempC = masterSensor.temperature(R_NOMINAL_PT100, R_REF_PT100);

  // 2. Verificación de fallos
  if (checkAndLogFault(masterSensor, "MASTER Sensor")) {
    return SENSOR_ERROR_VALUE;
  }

  // 3. Aplicar compensación de calibración maestra (si aplica)
  tempC += settings.getMasterOffset();

  // 4. Aplicar conversión de escala (Fahrenheit)
  return applyScaleConversion(tempC);
}

// ***************************************************************
// LECTURA: Sensor de Prueba
// ***************************************************************
float Sensors::readTestTemperature() {
  float r_ref = (settings.getSensorType() == SensorType::PT100) ? R_REF_PT100
                                                                : R_REF_PT1000;
  float nominal_res = (settings.getSensorType() == SensorType::PT100)
                          ? R_NOMINAL_PT100
                          : R_NOMINAL_PT1000;

  // 1. Lectura de temperatura
  float tempC = testSensor.temperature(nominal_res, r_ref);

  // 2. Verificación de fallos
  if (checkAndLogFault(testSensor, "TEST Sensor")) {
    return SENSOR_ERROR_VALUE;
  }

  // 3. Aplicar compensación de calibración maestra (si aplica)
  tempC += settings.getTestOffset();

  // 4. Aplicar conversión de escala (Fahrenheit)
  return applyScaleConversion(tempC);
}

// ***************************************************************
// CONFIGURACIÓN: Sensor de Prueba
// ***************************************************************
void Sensors::configureTestSensor(SensorType type, int wires) {
  settings.setSensorType(type);
  settings.setSensorWires(wires);

  // Configurar el chip MAX31865
  switch (wires) {
  case 2:
    testSensor.begin(MAX31865_2WIRE);
    break;
  case 3:
    testSensor.begin(MAX31865_3WIRE);
    break;
  case 4:
    testSensor.begin(MAX31865_4WIRE);
    break;
  default:
    logln("WARNING: Invalid wire count, defaulting to 3-wire.");
    testSensor.begin(MAX31865_3WIRE);
    break;
  }
}

// ***************************************************************
// LECTURA FILTRADA: Sensor Maestro (para el PID) aplicando EMA
// ***************************************************************
float Sensors::getFilteredMasterTemperature(float alpha) {
  // 1. Obtener la lectura bruta (ya aplica offset, fallos y conversión de
  // escala)
  float currentReading = readMasterTemperature(); // X_n

  // 2. Manejar valores de error (Robustez)
  if (currentReading == SENSOR_ERROR_VALUE) {
    // Si la lectura es inválida, devolvemos el último valor filtrado válido.
    return _emaMasterTemperature;
  }

  // 3. Inicialización del filtro (Si es la primera lectura válida)
  if (_emaMasterTemperature == SENSOR_ERROR_VALUE) {
    _emaMasterTemperature = currentReading;
    return _emaMasterTemperature;
  }

  // 4. Aplicar la Ecuación del EMA
  // Y_n = (alpha * X_n) + ( (1 - alpha) * Y_{n-1} )
  _emaMasterTemperature =
      (alpha * currentReading) + ((1.0f - alpha) * _emaMasterTemperature);

  // Devolver el valor filtrado
  return _emaMasterTemperature;
}

// ***************************************************************
// LECTURA FILTRADA: Sensor de Prueba (EMA)
// ***************************************************************
float Sensors::getFilteredTestTemperature(float alpha) {
  // 1. Obtener la lectura bruta (ya aplica offset, fallos y conversión de
  // escala)
  float currentReading = readTestTemperature(); // X_n

  // 2. Manejar valores de error (Robustez)
  if (currentReading == SENSOR_ERROR_VALUE) {
    // Si la lectura es inválida, devolvemos el último valor filtrado válido.
    return _emaTestTemperature;
  }

  // 3. Inicialización del filtro (Si es la primera lectura válida)
  if (_emaTestTemperature == SENSOR_ERROR_VALUE) {
    _emaTestTemperature = currentReading;
    return _emaTestTemperature;
  }

  // 4. Aplicar la Ecuación del EMA
  // Y_n = (alpha * X_n) + ( (1 - alpha) * Y_{n-1} )
  _emaTestTemperature =
      (alpha * currentReading) + ((1.0f - alpha) * _emaTestTemperature);

  // Devolver el valor filtrado
  return _emaTestTemperature;
}