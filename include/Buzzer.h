#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

// Definiciones de los tipos de sonido para la aplicación
enum class BeepType {
    READY,              // Inicialización completada (Doble bip rápido)
    TARGET_REACHED,     // Temperatura alcanzada (Tono largo y constante)
    ALARM,              // Error grave o límite superado (Triple bip fuerte)
    POINT_REGISTERED,   // Un toque corto (Registro de dato)
    START_PROCESS,      // Nuevo sonido: Proceso de calibración iniciado (Tono ascendente)
    ERROR_SILENT,       // Nuevo sonido: Fallo de sensor no crítico (Tono muy bajo y corto)
    STOP                // Nuevo sonido: Detener proceso actual (Tono descendente)
};

// Estructura para definir los pasos de una secuencia de bips
struct BeepStep {
    uint16_t frequency; // Frecuencia del tono (Hz)
    uint16_t duration;  // Duración del tono (ms)
};

const int BUZZER_PIN = 26;

class Buzzer {
public:
    // Constructor: Recibe el pin y el canal PWM a usar
    //Buzzer();
    
    // Configuración: Inicializa el canal PWM (llamar en setup())
    void begin();

    // Método principal: Inicia la reproducción del patrón de sonido
    void beep(BeepType type);

    // Método No-Bloqueante: Debe ser llamado repetidamente en loop()
    void handle();

    // Métodos de control
    void stop();
    void mute(bool enable);

private:
    const int buzzerPwmChannel = 2;
    const int pwmFrequency = 5000;
    const int pwmResolution = 8;

    // Variables de estado No-Bloqueante
    bool _isPlaying = false;
    bool _isMuted = false;
    uint32_t _lastTime = 0;
    
    const BeepStep* _currentPattern = nullptr;
    size_t _patternIndex = 0;
    size_t _patternSize = 0;

    // Patrones de sonido (Definiciones de las secuencias de BeepStep)
    const BeepStep* getPattern(BeepType type, size_t& size);

    void setTone(uint16_t freq, uint16_t duration);
};

#endif // BUZZER_H