#include "Heater.h"
#include <Arduino.h>

// Constructor (asumido si lo tuvieras, no incluido aquí)

/**
 * @brief Inicializa los pines y los canales PWM del ESP32.
 */
void Heater::begin() {
    // 1. Configuración de Canales PWM
    ledcSetup(heatPwmChannel, pwmFrequency, pwmResolution); // Canal 0 para Calentar (Pin 14)
    ledcAttachPin(PWM_HEATER_PIN, heatPwmChannel);
    
    ledcSetup(coolPwmChannel, pwmFrequency, pwmResolution); // Canal 1 para Enfriar (Pin 27)
    ledcAttachPin(PWM_COOLING_PIN, coolPwmChannel);

    // 2. Configuración de Pines de Habilitación (ENABLE)
    pinMode(EN_HEATER_PIN, OUTPUT);
    pinMode(EN_COOLING_PIN, OUTPUT);
    
    // **CRÍTICO:** Habilitar ambos lados del driver (R_EN y L_EN)
    // El driver está ahora "listo" para aceptar comandos PWM.
    digitalWrite(EN_HEATER_PIN, LOW); 
    digitalWrite(EN_COOLING_PIN, LOW);

    // 3. Detener ambos al inicio (PWM en 0)
    stop();
    Serial.println("Heater/Cooler initialized with PWM Dual (5kHz).");
    // Serial.println("R_EN (Pin 22) and L_EN (Pin 21) set to HIGH.");
}

/**
 * @brief Aplica potencia al elemento calefactor (RPWM).
 * @param dutyCycle Potencia de 0 (apagado) a 100 (máximo).
 */
void Heater::setHeat(float dutyCycle) {
    if (dutyCycle < 0) dutyCycle = 0;
    if (dutyCycle > 100) dutyCycle = 100;

    uint32_t maxDuty = (1 << pwmResolution) - 1; // 255
    // Usamos el valor real de tu umbral
    const uint32_t MIN_DUTY_THRESHOLD = 12; 

    uint32_t duty = 0;
    
    if (dutyCycle > 0.0f) {
        // 1. Calcular el valor lineal (0 - 255)
        float linear_duty = (dutyCycle / 100.0f) * (float)maxDuty;

        // 2. Aplicar el umbral y Mapeo Acelerado (CORRECCIÓN CLAVE)
        if (linear_duty > (float)MIN_DUTY_THRESHOLD) {
            // Si el Duty es mayor que el umbral (PID pide mucha potencia), 
            // no hacemos nada, solo limitamos a MAX.
            duty = (uint32_t)linear_duty;
        } else if (linear_duty > 0.0f) {
            // Si el Duty está entre 0 y MIN_DUTY_THRESHOLD, lo remapeamos/forzamos:
            // Opción A (La que tenías, pero corregida y más simple): Forzar el umbral mínimo
            // Esto causa la discontinuidad, pero es simple.
            duty = MIN_DUTY_THRESHOLD;
        
            /* // Opción B (Mapeo Lineal Acelerado, recomendado para precisión)
            // Esto re-mapea el rango (0% a 4.7%) a un rango más útil (12 a 255)
            // y requiere ajustar tu Kp, Ki, Kd, pero ahora ya están sintonizados 
            // con el defecto, así que usemos la Opción A corregida.
            */
        }
        
        if (duty > maxDuty) duty = maxDuty; // Cap (solo por seguridad)
    }

    // ... (rest of the code for digitalWrite, ledcWrite, Serial.printf, etc.)
    // La mutua exclusión y llamadas a ledcWrite son correctas.
    digitalWrite(EN_HEATER_PIN, HIGH);
    digitalWrite(EN_COOLING_PIN, HIGH);

    ledcWrite(coolPwmChannel, 0); // MUTUA EXCLUSIÓN

    Serial.printf("Heater set to %.2f%% power (Duty: %u)\n", dutyCycle, duty);
    ledcWrite(heatPwmChannel, duty);
}

/**
 * @brief Aplica potencia al elemento de enfriamiento (LPWM).
 * @param power Potencia de 0 (apagado) a 100 (máximo).
 */
void Heater::setCool(float dutyCycle) {
    // 1. Capado de seguridad
    if (dutyCycle < 0.0f) dutyCycle = 0.0f;
    if (dutyCycle > 100.0f) dutyCycle = 100.0f;

    // 2. Definición de constantes
    uint32_t maxDuty = (1 << pwmResolution) - 1; // 255
    const uint32_t MIN_DUTY_THRESHOLD = 12; // Tu umbral de arranque confirmado (Duty 12)

    uint32_t duty = 0;

    if (dutyCycle > 0.0f) {
        
        // 3. Calcular el valor lineal (0 - 255)
        float linear_duty = (dutyCycle / 100.0f) * (float)maxDuty;

        // 4. Aplicar el umbral: Si el duty calculado es bajo, forzar el mínimo.
        duty = (uint32_t)linear_duty;
        
        if (duty > 0 && duty < MIN_DUTY_THRESHOLD) {
             // Esta lógica fuerza el encendido a Duty 12 si la potencia es > 0 y baja.
             duty = MIN_DUTY_THRESHOLD;
        }
        
        if (duty > maxDuty) duty = maxDuty; // Cap de seguridad (Duty 255)
    }

    // 5. Mutua Exclusión y Aplicación
    // Aseguramos que los pines de habilitación (ENABLE) estén activos si se usan.
    digitalWrite(EN_HEATER_PIN, HIGH);
    digitalWrite(EN_COOLING_PIN, HIGH);

    // MUTUA EXCLUSIÓN: Asegurar que el lado de Calentamiento esté en 0
    ledcWrite(heatPwmChannel, 0); 

    Serial.printf("Cooling set to %.2f%% power (Duty: %u)\n", dutyCycle, duty);

    // Aplicar PWM al lado de Enfriamiento
    ledcWrite(coolPwmChannel, duty);
}

/**
 * @brief Detiene completamente los elementos de calentamiento y enfriamiento.
 */
void Heater::stop() {
    // Solo necesitamos poner ambos PWM en 0 (L_EN y R_EN siguen en HIGH)
    ledcWrite(heatPwmChannel, 0); 
    ledcWrite(coolPwmChannel, 0); 
    
    // **OPCIONAL PERO RECOMENDADO:** Deshabilitar el driver completamente para mayor seguridad.
    // Si el driver tiene un modo de "Inhibit", es mejor usarlo. Si no, poner los EN en LOW.
    digitalWrite(EN_HEATER_PIN, LOW); 
    digitalWrite(EN_COOLING_PIN, LOW); 
    
    // Serial.println("Heater/Cooler stopped.");
}