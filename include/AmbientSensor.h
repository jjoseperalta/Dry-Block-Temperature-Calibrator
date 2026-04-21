#ifndef AMBIENTSENSOR_H
#define AMBIENTSENSOR_H

#include <OneWire.h>
#include <DallasTemperature.h>

class AmbientSensor {
public:
    AmbientSensor(uint8_t pin = 5);
    
    void begin();
    
    // Lee la temperatura con filtro y offset
    float readTemperature();
    
    // Getters
    float getLastTemperature() const { return _emaTemp; }
    void setOffset(float offset) { _offset = offset; }

private:
    OneWire _oneWire;
    DallasTemperature _sensors;
    
    float _emaTemp;    // Temperatura filtrada
    float _offset;     // Ajuste de calibración
    float _alpha;      // Factor de suavizado (0.0 a 1.0)
    
    const float ERROR_VALUE = -999.0f;
};

#endif