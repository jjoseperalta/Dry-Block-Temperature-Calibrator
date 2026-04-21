#include "AmbientSensor.h"
#include "Logger.h"

AmbientSensor::AmbientSensor(uint8_t pin) 
    : _oneWire(pin), 
      _sensors(&_oneWire), 
      _emaTemp(ERROR_VALUE), 
      _offset(0.0f), 
      _alpha(0.1f) {} // Alpha 0.1 significa que la lectura actual influye un 10%

void AmbientSensor::begin() {
    _sensors.begin();
    _sensors.setResolution(12); // Máxima resolución: saltos de 0.0625°C
    _sensors.setWaitForConversion(false);
    
    _sensors.requestTemperatures();
}

float AmbientSensor::readTemperature() {
    float rawTemp = _sensors.getTempCByIndex(0);
    
    // Validar conexión
    if (rawTemp == DEVICE_DISCONNECTED_C) {
        logln("ERROR: DS18B20 desconectado");
        return _emaTemp; // Devolvemos el último valor válido
    }

    // Aplicar Offset de calibración
    float currentReading = rawTemp + _offset;

    // Inicializar filtro si es la primera lectura
    if (_emaTemp == ERROR_VALUE) {
        _emaTemp = currentReading;
    } else {
        // Aplicar Filtro EMA (igual que en Sensors.cpp)
        // Y_n = (alpha * X_n) + ((1 - alpha) * Y_{n-1})
        _emaTemp = (_alpha * currentReading) + ((1.0f - _alpha) * _emaTemp);
    }
    
    // Disparar la siguiente conversión para que esté lista en la próxima llamada
    _sensors.requestTemperatures();
    
    return _emaTemp;
}