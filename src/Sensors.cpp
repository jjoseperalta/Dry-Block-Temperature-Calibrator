#include "Sensors.h"
#include "Logger.h"
#include <Arduino.h>

// Sensors::Sensors(Settings &settings)
//     : settings(settings), masterSensor(Adafruit_MAX31865(
//                               MAX_CS_MASTER, MAX_MOSI, MAX_MISO, MAX_SCK)),
//       testSensor(Adafruit_MAX31865(MAX_CS_TEST, MAX_MOSI, MAX_MISO, MAX_SCK)) {}

Sensors::Sensors(Settings &settings)
    : settings(settings), masterSensor(Adafruit_MAX31865(MAX_CS_MASTER)),
      testSensor(Adafruit_MAX31865(MAX_CS_TEST)) {}

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
    sensor.clearFault();
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
float Sensors::getRawMasterTemperature() {
  // 1. Verificación de fallos
  if (checkAndLogFault(masterSensor, "MASTER Sensor")) {
    return SENSOR_ERROR_VALUE;
  }

  // 2. Leer RTD RAW
  uint16_t rtdRaw = masterSensor.readRTD();

  // 3. Convertir a resistencia
  float resistance = (rtdRaw * R_REF_PT100) / 32768.0f;

  // 4. Validación rápida de rango físico
  if (resistance < 80.0f || resistance > 400.0f) {
    return SENSOR_ERROR_VALUE;
  }
  
  // 5. Lectura de temperatura
  float tempC = masterSensor.temperature(R_NOMINAL_PT100, R_REF_PT100);

  return tempC;
}

float Sensors::getMasterTemperature() {
  // float tempC = getRawMasterTemperature();
  float tempC = getFilteredMasterTemperature(0.1f, true);

  if (tempC != SENSOR_ERROR_VALUE) {
    float correction = getMasterCorrectionAtTemp(tempC);
    tempC += correction;
  }

  return tempC;
}

// ***************************************************************
// LECTURA: Sensor de Prueba
// ***************************************************************
float Sensors::getRawTestTemperature() {
  float r_ref = (settings.getSensorType() == SensorType::PT100) ? R_REF_PT100
                                                                : R_REF_PT1000;
  float nominal_res = (settings.getSensorType() == SensorType::PT100)
                          ? R_NOMINAL_PT100
                          : R_NOMINAL_PT1000;

  // 1. Verificación de fallos
  if (checkAndLogFault(testSensor, "TEST Sensor")) {
    return SENSOR_ERROR_VALUE;
  }

  // 2. Leer RTD RAW
  uint16_t rtdRaw = testSensor.readRTD();

  // 3. Convertir a resistencia
  float resistance = (rtdRaw * r_ref) / 32768.0f;

  // 4. Validación rápida de rango físico
  if (resistance < 80.0f || resistance > 400.0f) {
    return SENSOR_ERROR_VALUE;
  }
  
  // 5. Lectura de temperatura
  float tempC = testSensor.temperature(nominal_res, r_ref);

  return tempC;
}

float Sensors::getTestTemperature() {
  // float tempC = getRawTestTemperature();
  // float tempC = getFilteredTestTemperature(0.1f, true);

  // if (tempC != SENSOR_ERROR_VALUE) {
  //   float correction = getTestCorrectionAtTemp(tempC);
  //   tempC += correction;
  // }

  // return tempC;

  return generateTestTemperature();
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
// FILTRO EMA: aplicando EMA
// ***************************************************************
float Sensors::applyEMAFilter(float currentReading, float& emaValue, float alpha) {
    // Lectura inválida
    if (currentReading == SENSOR_ERROR_VALUE)
        return emaValue;

    // Primera lectura válida
    if (emaValue == SENSOR_ERROR_VALUE){
        emaValue = currentReading;
        return emaValue;
    }

    // EMA Y_n = (alpha * X_n) + ( (1 - alpha) * Y_{n-1} )
    float filtered = (alpha * currentReading) + ((1.0f - alpha) * emaValue);

    // Histéresis anti-ruido
    if (fabsf(filtered - emaValue) > 0.04f)
        emaValue = filtered;

    return emaValue;
}

// ***************************************************************
// LECTURA FILTRADA: Sensor Maestro (para el PID) aplicando EMA
// ***************************************************************
float Sensors::getFilteredMasterTemperature(float alpha, bool useRaw) {
  float currentReading;
  
  // if (useRaw) {
  //   // RAW: Sin offset, sin conversión de escala (siempre Celsius)
  //   currentReading = getRawMasterTemperature();
  // } else {
  //   // Compensado: Con offset y conversión de escala
  //   currentReading = getMasterTemperature();
  // }
  currentReading = getRawMasterTemperature();
  
  return applyEMAFilter(currentReading, _emaMasterTemperature, alpha);
}

// ***************************************************************
// LECTURA FILTRADA: Sensor de Prueba (EMA)
// ***************************************************************
float Sensors::getFilteredTestTemperature(float alpha, bool useRaw) {
  float currentReading;
  
  // if (useRaw) {
  //   // RAW: Sin offset, sin conversión de escala (depende del tipo de sensor)
  //   currentReading = getRawTestTemperature();
  // } else {
  //   // Compensado: Con offset y conversión de escala
  //   currentReading = getTestTemperature();
  // }
  currentReading = getRawTestTemperature();
  
  return applyEMAFilter(currentReading, _emaTestTemperature, alpha);
}

float Sensors::interpolateCorrection(float temp, const float corrections[3]) {
  // Obtener puntos de calibración
  float p1 = settings.getCalibrationPoint(0);
  float p2 = settings.getCalibrationPoint(1);
  float p3 = settings.getCalibrationPoint(2);
  
  // Si no hay correcciones válidas
  if (corrections[0] == 0 && corrections[1] == 0 && corrections[2] == 0) {
    return 0.0f;
  }
  
  // Interpolación
  if (temp <= p1) {
    return corrections[0];
  } else if (temp <= p2) {
    float t = (temp - p1) / (p2 - p1);
    return corrections[0] + t * (corrections[1] - corrections[0]);
  } else if (temp <= p3) {
    float t = (temp - p2) / (p3 - p2);
    return corrections[1] + t * (corrections[2] - corrections[1]);
  } else {
    return corrections[2];
  }
}

float Sensors::getMasterCorrectionAtTemp(float temp) {
  float corrections[3];
  corrections[0] = settings.getMasterCorrections(0);
  corrections[1] = settings.getMasterCorrections(1);
  corrections[2] = settings.getMasterCorrections(2);
  return interpolateCorrection(temp, corrections);
}

float Sensors::getTestCorrectionAtTemp(float temp) {
  float corrections[3];
  corrections[0] = settings.getTestCorrections(0);
  corrections[1] = settings.getTestCorrections(1);
  corrections[2] = settings.getTestCorrections(2);
  return interpolateCorrection(temp, corrections);
}

void Sensors::calculateCorrectionsFromErrors(const float masterErrors[3], const float testErrors[3]) {
  for (int i = 0; i < 3; i++) {
    // La corrección es el NEGATIVO del error
    // Error = real - medido → Corrección = medido + X = real → X = real - medido = error
    settings.setMasterCorrections(i, -masterErrors[i]);
    settings.setTestCorrections(i, -testErrors[i]);
    
    logf("Point %d (%.1f°C): Master error=%.3f → correction=%.3f, Test error=%.3f → correction=%.3f\n",
         i, settings.getCalibrationPoint(i), 
         masterErrors[i], -masterErrors[i],
         testErrors[i], -testErrors[i]);
  }
  settings.save();
}

float Sensors::generateTestTemperature() {
    float masterC = getMasterTemperature(); // Ejemplo: 23.00 °C
    
    // Tolerancia equivalente a +-0.45 °F -> es decir, +-0.25 °C
    const float toleranciaC = 0.15f;
    
    // 1. Generamos un factor aleatorio flotante entre 0.0 y 1.0
    float r = (float)random(0, 10000) / 10000.0f;
    
    // 2. Escalamos el aleatorio para que vaya desde -toleranciaC hasta +toleranciaC
    // (r * 2.0f - 1.0f) va de -1.0 a +1.0
    float variacionC = (r * 2.0f - 1.0f) * toleranciaC; // Dará entre -0.25°C y +0.25°C
    
    // 3. Sumamos la variación a la temperatura base
    float tempC = masterC + variacionC;
    
    return tempC;
}