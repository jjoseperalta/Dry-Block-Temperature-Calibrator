#include "Heater.h"
#include "Logger.h"
#include <Arduino.h>

static MicroPWMConfig microFine = {
    .minPower = 6.2f, // ajustable 6.0–7.0 %
    .periodMs = 8000, // 5 segundos
    .onTimeMs = 400   // 10 % duty térmico
};

/**
 * @brief Inicializa los pines y los canales PWM del ESP32.
 */
void Heater::begin() {
  // 1. Configuración de Canales PWM
  ledcSetup(heatPwmChannel, pwmFrequency,
            pwmResolution); // Canal 0 para Calentar (Pin 14)
  ledcAttachPin(PWM_HEATER_PIN, heatPwmChannel);

  ledcSetup(coolPwmChannel, pwmFrequency,
            pwmResolution); // Canal 1 para Enfriar (Pin 27)
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
  logln("Heater/Cooler initialized with PWM Dual (5kHz).");
  // logln("R_EN (Pin 22) and L_EN (Pin 21) set to HIGH.");
}

void Heater::setPower(float dutyCycle, bool fineZone) {
  dutyCycle = constrain(dutyCycle, -100.0f, 100.0f);

  // --------------------------------------------------
  // Micro-PWM térmico metrológico (zona fina)
  // --------------------------------------------------
  if (fineZone && dutyCycle > 0.0f && dutyCycle < microFine.minPower) {

    static uint32_t t0 = 0;
    uint32_t now = millis();

    if (t0 == 0)
      t0 = now;

    uint32_t phase = (now - t0) % microFine.periodMs;

    if (phase < microFine.onTimeMs) {
      dutyCycle = microFine.minPower;
    } else {
      dutyCycle = 0.0f;
    }
  }

  // --------------------------------------------------
  // Mínimo efectivo físico (después del dither)
  // --------------------------------------------------
  dutyCycle = applyMinimumEffectivePower(dutyCycle, fineZone);

  // --------------------------------------------------
  // Actuación final
  // --------------------------------------------------
  if (dutyCycle > 0.0f) {
    setCool(0);
    setHeat(dutyCycle);

  } else if (dutyCycle < 0.0f) {
    float coolPower = min(-dutyCycle, 80.0f);
    setHeat(0);
    setCool(coolPower);

  } else {
    stop();
  }
}

float Heater::applyMinimumEffectivePower(float dutyCycle, bool fineZone) {
  const float MIN_HEAT_COARSE = settings.getMinHeatPower(); // ej. 6.5 %
  const float MIN_HEAT_FINE = 4.5f;                         // empezar aquí
  const float MIN_COOL = settings.getMinCoolPower();        // ej. 10 %

  float minHeat = fineZone ? MIN_HEAT_FINE : MIN_HEAT_COARSE;

  if (dutyCycle > 0.0f && dutyCycle < minHeat)
    return minHeat;

  if (dutyCycle < 0.0f && dutyCycle > -MIN_COOL)
    return -MIN_COOL;

  return dutyCycle;
}

void Heater::setHeat(float dutyCycle) {
  // 1. Capado de seguridad (0% - 100%)
  if (dutyCycle < 0.0f)
    dutyCycle = 0.0f;
  if (dutyCycle > 100.0f)
    dutyCycle = 100.0f;

  uint32_t maxDuty = (1 << pwmResolution) - 1; // 255 (Asumiendo 8 bits)

  // **NOTA DE LA CORRECCIÓN:** Se elimina la variable MIN_DUTY_THRESHOLD
  // y la lógica de umbral condicional para lograr un control lineal.

  uint32_t duty = 0;

  if (dutyCycle > 0.0f) {
    // 1. Calcular el valor lineal (0 - 255)
    float linear_duty = (dutyCycle / 100.0f) * (float)maxDuty;

    // 2. Asignar el Duty directamente (Control Lineal)
    duty = (uint32_t)linear_duty;
  }

  if (duty > maxDuty)
    duty = maxDuty; // Cap final (solo por seguridad)

  // La mutua exclusión y llamadas a ledcWrite son correctas.
  // Asumimos que EN_HEATER_PIN y EN_COOLING_PIN son pines de habilitación
  // general.
  digitalWrite(EN_HEATER_PIN, HIGH);
  digitalWrite(EN_COOLING_PIN, HIGH);

  // MUTUA EXCLUSIÓN: Asegurar que el lado de Enfriamiento esté en 0
  ledcWrite(coolPwmChannel, 0);

  if (duty > 0)
    logf("Heater set to %.2f%% power (Duty: %u)\n", dutyCycle, duty);

  // Aplicar PWM al lado de Calentamiento
  ledcWrite(heatPwmChannel, duty);
}

void Heater::setCool(float dutyCycle) {
  // 1. Capado de seguridad (0% - 100%)
  if (dutyCycle < 0.0f)
    dutyCycle = 0.0f;
  if (dutyCycle > 100.0f)
    dutyCycle = 100.0f;

  // 2. Definición de constantes
  uint32_t maxDuty = (1 << pwmResolution) - 1; // 255 (Asumiendo 8 bits)

  // **NOTA DE LA CORRECCIÓN:** Se elimina la variable MIN_DUTY_THRESHOLD
  // y la lógica de umbral condicional para lograr un control lineal.

  uint32_t duty = 0;

  if (dutyCycle > 0.0f) {

    // 3. Calcular el valor lineal (0 - 255)
    float linear_duty = (dutyCycle / 100.0f) * (float)maxDuty;

    // 4. Asignar el Duty directamente (Control Lineal)
    duty = (uint32_t)linear_duty;
  }

  if (duty > maxDuty)
    duty = maxDuty; // Cap de seguridad (Duty 255)

  // 5. Mutua Exclusión y Aplicación
  digitalWrite(EN_HEATER_PIN, HIGH);
  digitalWrite(EN_COOLING_PIN, HIGH);

  // MUTUA EXCLUSIÓN: Asegurar que el lado de Calentamiento esté en 0
  ledcWrite(heatPwmChannel, 0);

  if (duty > 0)
    logf("Cooling set to %.2f%% power (Duty: %u)\n", dutyCycle, duty);

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

  // **OPCIONAL PERO RECOMENDADO:** Deshabilitar el driver completamente para
  // mayor seguridad. Si el driver tiene un modo de "Inhibit", es mejor usarlo.
  // Si no, poner los EN en LOW.
  digitalWrite(EN_HEATER_PIN, LOW);
  digitalWrite(EN_COOLING_PIN, LOW);

  // logln("Heater/Cooler stopped.");
}

void Heater::hold(float temp, float setpoint) {
  static float holdPWM = 8.0f; // valor inicial seguro

  const float STEP = 0.15f;    // ajuste fino
  const float DEAD = 0.02f;    // banda muerta térmica

  if (temp < setpoint - DEAD) {
    holdPWM += STEP;
  }
  else if (temp > setpoint + DEAD) {
    holdPWM -= STEP;
  }

  holdPWM = constrain(holdPWM, 0.0f, 15.0f);

  setHeat(holdPWM);
}
