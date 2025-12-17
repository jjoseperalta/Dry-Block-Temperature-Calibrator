#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_MAX31865.h>
#include "Settings.h"

// Valor de retorno para indicar una lectura fallida del sensor
const float SENSOR_ERROR_VALUE = -999.0f;

// Pin definitions
const int MAX_MOSI = 23;
const int MAX_MISO = 19;
const int MAX_SCK = 18;
const int MAX_CS_MASTER = 4;
const int MAX_CS_TEST = 32;

// Reference resistor for PT100
const float R_REF_PT100 = 430.0;
// Reference resistor for PT1000
const float R_REF_PT1000 = 4300.0;

// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
const float R_NOMINAL_PT100 = 100.0;
const float R_NOMINAL_PT1000 = 1000.0;

class Sensors {
public:
    Sensors(Settings& settings);
    void begin();
    float readMasterTemperature();
    float readTestTemperature();
    void configureTestSensor(SensorType type, int wires);
    float getFilteredMasterTemperature(float alpha = 0.1f);
    float getFilteredTestTemperature(float alpha = 0.1f);

private:
    Settings& settings;
    Adafruit_MAX31865 masterSensor;
    Adafruit_MAX31865 testSensor;

    float _emaMasterTemperature = SENSOR_ERROR_VALUE;
    float _emaTestTemperature = SENSOR_ERROR_VALUE;

    // Helper para la conversión de escala
    float applyScaleConversion(float tempC);
    
    // Helper para verificar y loguear fallos del MAX31865
    bool checkAndLogFault(Adafruit_MAX31865& sensor, const String& sensorName);
};

#endif // SENSORS_H
