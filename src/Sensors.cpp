#include "Sensors.h"

Sensors::Sensors(Settings& settings) :
    settings(settings),
    masterSensor(Adafruit_MAX31865(MAX_CS_MASTER, MAX_MOSI, MAX_MISO, MAX_SCK)),
    testSensor(Adafruit_MAX31865(MAX_CS_TEST, MAX_MOSI, MAX_MISO, MAX_SCK)) {}

void Sensors::begin() {
    masterSensor.begin(MAX31865_3WIRE);
    Serial.println("Master Sensor Initialized (CS: " + String(MAX_CS_MASTER) + ")");

    // The test sensor is configured based on saved settings
    configureTestSensor(settings.getSensorType(), settings.getSensorWires());
    Serial.println("Test Sensor Initialized (CS: " + String(MAX_CS_TEST) + ")");
}

// ***************************************************************
// HELPER: Verifica y loguea los fallos del MAX31865
// ***************************************************************
bool Sensors::checkAndLogFault(Adafruit_MAX31865& sensor, const String& sensorName) {
    uint8_t fault = sensor.readFault();
    if (fault) {
        Serial.print("ERROR: ");
        Serial.print(sensorName);
        Serial.print(" Fault Detected (0x");
        Serial.print(fault, HEX);
        Serial.print("): ");
        
        if (fault & MAX31865_FAULT_HIGHTHRESH) Serial.print("High Threshold, ");
        if (fault & MAX31865_FAULT_LOWTHRESH) Serial.print("Low Threshold, ");
        if (fault & MAX31865_FAULT_REFINLOW) Serial.print("REFIN Low, ");
        if (fault & MAX31865_FAULT_REFINHIGH) Serial.print("REFIN High, ");
        if (fault & MAX31865_FAULT_RTDINLOW) Serial.print("RTD Low, ");
        if (fault & MAX31865_FAULT_OVUV) Serial.print("Over/Under Voltage");
        Serial.println();
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
    // En un sistema real, el sensor maestro debe ser PT100 o PT1000 fijo para ser una referencia.
    // Usamos PT100 como estándar de referencia para el MASTER.
    
    // 1. Lectura de temperatura
    float tempC = masterSensor.temperature(R_NOMINAL_PT100, R_REF_PT100);
    
    // 2. Verificación de fallos
    if (checkAndLogFault(masterSensor, "MASTER Sensor")) {
        return SENSOR_ERROR_VALUE;
    }

    // 3. Aplicar compensación de calibración maestra (si aplica)
    tempC += settings.getMasterCalibrationOffset();
    
    // 4. Aplicar conversión de escala (Fahrenheit)
    return applyScaleConversion(tempC);
}

// ***************************************************************
// LECTURA: Sensor de Prueba
// ***************************************************************
float Sensors::readTestTemperature() {
    // Los valores de referencia y nominales dependen de la configuración guardada
    float r_ref = (settings.getSensorType() == SensorType::PT100) ? R_REF_PT100 : R_REF_PT1000;
    float nominal_res = (settings.getSensorType() == SensorType::PT100) ? R_NOMINAL_PT100 : R_NOMINAL_PT1000;
    
    // 1. Lectura de temperatura
    float tempC = testSensor.temperature(nominal_res, r_ref);
    
    // 2. Verificación de fallos
    if (checkAndLogFault(testSensor, "TEST Sensor")) {
        return SENSOR_ERROR_VALUE;
    }

    // 3. Aplicar conversión de escala (Fahrenheit)
    return applyScaleConversion(tempC);
}

// ***************************************************************
// CONFIGURACIÓN: Sensor de Prueba
// ***************************************************************
void Sensors::configureTestSensor(SensorType type, int wires) {
    // Actualizar settings (esto es solo por si se llama desde un comando, asegura que los valores están actualizados)
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
            Serial.println("WARNING: Invalid wire count, defaulting to 3-wire.");
            testSensor.begin(MAX31865_3WIRE);
            break;
    }
}