#include "Buzzer.h"

// Definición de Patrones de Sonido (Secuencias de Frecuencia y Duración)
// NOTA: La duración 0 en la frecuencia indica una pausa (silencio).

// READY: Doble bip rápido (440Hz -> 550Hz)
const BeepStep readyPattern[] = {
    // Frecuencia, Duración, Volumen
    {440, 100, 90}, // Bip 1 (Frecuencia base)
    {0, 80, 0},     // Pausa corta
    {550, 120, 90}, // Bip 2 (Frecuencia más alta y ligeramente más larga)
    {0, 1, 0}       // Silencio y final
};

// TARGET_REACHED: Tono largo y constante (880Hz) con ataque/decaimiento suave
const BeepStep targetReachedPattern[] = {
    // Frecuencia, Duración, Volumen
    {880, 20, 50},  // Rampa inicial (Attack)
    {880, 400, 100},// Cuerpo
    {880, 50, 60},  // Rampa final (Release)
    {880, 30, 30},  // Continuación del Release
    {0, 1, 0}       // Silencio y final
};

// ALARM: Triple bip fuerte (800Hz) con ligero 'pitch bend' para la urgencia
const BeepStep alarmPattern[] = {
    // Frecuencia, Duración, Volumen
    {800, 100, 100}, // Tono Fuerte 1
    {790, 100, 100}, // Tono Ligeramente más bajo 
    {0, 100, 0},     // Pausa
    {800, 100, 100}, // Tono Fuerte 2
    {790, 100, 100}, // Tono Ligeramente más bajo
    {0, 100, 0},     // Pausa
    {800, 100, 100}, // Tono Fuerte 3
    {790, 100, 100}, // Tono Ligeramente más bajo
    {0, 1, 0}        // Silencio y final
};

// POINT_REGISTERED: Tono corto y firme (600Hz)
const BeepStep pointRegisteredPattern[] = {
    {600, 5, 100},  // Ataque rápido
    {500, 25, 90},  // Cuerpo del clic
    {0, 1, 0}       // Fin
};

// START_PROCESS: Tono ascendente más suave (440Hz -> 660Hz)
const BeepStep startProcessPattern[] = {
    {440, 30, 80},  
    {500, 30, 90},  
    {580, 30, 100}, 
    {660, 30, 100}, 
    {0, 1, 0}
};

// ERROR_SILENT: Dos tonos bajos y lentos (120Hz/100Hz)
const BeepStep errorSilentPattern[] = {
    {120, 100, 70}, 
    {0, 50, 0},     
    {100, 100, 70}, 
    {0, 1, 0}
};

// STOP: Tono descendente más suave (660Hz -> 440Hz)
const BeepStep stopPattern[] = {
    {660, 30, 100},
    {580, 30, 100},
    {500, 30, 90},
    {440, 30, 80},
    {0, 1, 0}
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

void Buzzer::setTone(uint16_t freq, uint16_t duration, uint8_t volume) {
    if (_isMuted || freq == 0 || volume == 0) { // Si el volumen es 0, también silenciamos
        ledcWriteTone(buzzerPwmChannel, 0);
        ledcWrite(buzzerPwmChannel, 0);
    } else {
        ledcWriteTone(buzzerPwmChannel, freq);
        
        // Calcular el ciclo de trabajo basado en el volumen (0-100)
        int maxDuty = (1 << pwmResolution) - 1;
        // duty = maxDuty * (volume / 100.0)
        int duty = (maxDuty * volume) / 100; 
        
        // Aseguramos un mínimo para evitar división por cero si la frecuencia es muy baja
        if (duty < 1) duty = 1; 

        ledcWrite(buzzerPwmChannel, duty);
    }
    _lastTime = millis() + duration;
}

void Buzzer::beep(BeepType type) {
    // 1. Detener cualquier sonido en curso.
    // Esto permite que los sonidos nuevos (como ALARM)
    // anulen inmediatamente los sonidos anteriores.
    if (_isPlaying) {
        stop(); 
    }
    
    // Si la función stop() es llamada, _isPlaying es false, por lo que
    // el resto del código procede con el nuevo sonido.
    
    // 2. Obtener y validar el patrón
    _currentPattern = getPattern(type, _patternSize);
    if (_currentPattern == nullptr) return;

    // 3. Inicializar el estado de reproducción
    _patternIndex = 0;
    _isPlaying = true;

    // 4. Inicia el primer paso (Ahora pasa el volumen)
    setTone(
        _currentPattern[_patternIndex].frequency, 
        _currentPattern[_patternIndex].duration, 
        _currentPattern[_patternIndex].volume
    );
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
    
    // Verifica si el paso actual ha terminado (Manejo correcto de wrap-around)
    if (millis() >= _lastTime) { 
        _patternIndex++;

        if (_patternIndex < _patternSize) {
            // Continúa con el siguiente tono/pausa
            setTone(_currentPattern[_patternIndex].frequency, _currentPattern[_patternIndex].duration, _currentPattern[_patternIndex].volume);
        } else {
            // El patrón ha terminado
            stop();
        }
    }
}