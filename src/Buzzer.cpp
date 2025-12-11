#include "Buzzer.h"

// Definición de Patrones de Sonido (Secuencias de Frecuencia y Duración)
// NOTA: La duración 0 en la frecuencia indica una pausa (silencio).

// READY: Doble bip rápido (440Hz)
const BeepStep readyPattern[] = {
    {440, 100}, {0, 100}, // Bip 1 y pausa
    {440, 100}, {0, 1}    // Bip 2 y final
};

// TARGET_REACHED: Tono largo y constante (880Hz)
const BeepStep targetReachedPattern[] = {
    {880, 500}, {0, 1} // Un solo bip largo
};

// ALARM: Triple bip fuerte (600Hz)
const BeepStep alarmPattern[] = {
    {600, 200}, {0, 200},
    {600, 200}, {0, 200},
    {600, 200}, {0, 1}
};

// POINT_REGISTERED: Tono corto (440Hz)
const BeepStep pointRegisteredPattern[] = {
    {440, 50}, {0, 1}
};

// START_PROCESS: Tono ascendente (440Hz -> 660Hz)
const BeepStep startProcessPattern[] = {
    {440, 100}, {0, 50}, 
    {550, 100}, {0, 50}, 
    {660, 100}, {0, 1}
};

// ERROR_SILENT: Tono de fallo de sensor no crítico (100Hz)
const BeepStep errorSilentPattern[] = {
    {100, 50}, {0, 1}
};

// STOP: Tono descendente (660Hz -> 440Hz)
const BeepStep stopPattern[] = {
    {660, 100}, {0, 50},
    {550, 100}, {0, 50},
    {440, 100}, {0, 1}
};

// ***************************************************************
// IMPLEMENTACIÓN DE LA CLASE
// ***************************************************************

//Buzzer::Buzzer() {}

void Buzzer::begin() {
    // La configuración LEDc (PWM) debe estar en el setupHardware, 
    // pero garantizamos que se adjunte el pin aquí.
    ledcSetup(buzzerPwmChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(BUZZER_PIN, buzzerPwmChannel);
    ledcWrite(buzzerPwmChannel, 0); // Asegura que esté silenciado al inicio
}

// Devuelve el patrón de secuencia y su tamaño
const BeepStep* Buzzer::getPattern(BeepType type, size_t& size) {
    switch (type) {
        case BeepType::READY:
            size = sizeof(readyPattern) / sizeof(readyPattern[0]);
            return readyPattern;
        case BeepType::TARGET_REACHED:
            size = sizeof(targetReachedPattern) / sizeof(targetReachedPattern[0]);
            return targetReachedPattern;
        case BeepType::ALARM:
            size = sizeof(alarmPattern) / sizeof(alarmPattern[0]);
            return alarmPattern;
        case BeepType::POINT_REGISTERED:
            size = sizeof(pointRegisteredPattern) / sizeof(pointRegisteredPattern[0]);
            return pointRegisteredPattern;
        case BeepType::START_PROCESS:
            size = sizeof(startProcessPattern) / sizeof(startProcessPattern[0]);
            return startProcessPattern;
        case BeepType::ERROR_SILENT:
            size = sizeof(errorSilentPattern) / sizeof(errorSilentPattern[0]);
            return errorSilentPattern;
        case BeepType::STOP:
            size = sizeof(stopPattern) / sizeof(stopPattern[0]);
            return stopPattern;
        default:
            size = 0;
            return nullptr;
    }
}

void Buzzer::setTone(uint16_t freq, uint16_t duration) {
    if (_isMuted || freq == 0) {
        // Stop tone and ensure silence
        ledcWriteTone(buzzerPwmChannel, 0);
        ledcWrite(buzzerPwmChannel, 0);
    } else {
        // Reproduce el tono (50% de ciclo de trabajo)
        ledcWriteTone(buzzerPwmChannel, freq);
        // Compute max duty from configured resolution and set ~50% duty
        int maxDuty = (1 << pwmResolution) - 1;
        int duty = maxDuty / 2;
        ledcWrite(buzzerPwmChannel, duty);
    }
    _lastTime = millis() + duration;
}

void Buzzer::beep(BeepType type) {
    if (_isPlaying) {
        // Evita interrumpir un sonido que ya está en curso
        return; 
    }
    
    _currentPattern = getPattern(type, _patternSize);
    if (_currentPattern == nullptr) return;

    _patternIndex = 0;
    _isPlaying = true;

    // Inicia el primer paso
    setTone(_currentPattern[_patternIndex].frequency, _currentPattern[_patternIndex].duration);
}

void Buzzer::stop() {
    _isPlaying = false;
    ledcWrite(buzzerPwmChannel, 0); // Silencia inmediatamente
    _patternIndex = 0;
    _currentPattern = nullptr;
}

void Buzzer::mute(bool enable) {
    _isMuted = enable;
    if (enable) {
        stop(); // Si se silencia, detenemos cualquier sonido inmediatamente
    }
}

// *** FUNCIÓN CRÍTICA NO-BLOQUEANTE ***
void Buzzer::handle() {
    if (!_isPlaying) {
        return;
    }
    
    // Verifica si el paso actual ha terminado
    if (millis() >= _lastTime) {
        // Avanza al siguiente paso del patrón
        _patternIndex++;

        if (_patternIndex < _patternSize) {
            // Continúa con el siguiente tono/pausa
            setTone(_currentPattern[_patternIndex].frequency, _currentPattern[_patternIndex].duration);
        } else {
            // El patrón ha terminado
            stop();
        }
    }
}