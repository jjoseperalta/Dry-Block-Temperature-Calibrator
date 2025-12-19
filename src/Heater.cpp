#include "Heater.h"
#include "Logger.h"
#include <Arduino.h>

void Heater::begin() {
  microFine.minPower = 6.1f;
  microFine.periodMs = 4000;
  microFine.onTimeMs = 400;

  ledcSetup(heatPwmChannel, pwmFrequency,
            pwmResolution); // Canal 0 para Calentar (Pin 14)
  ledcAttachPin(PWM_HEATER_PIN, heatPwmChannel);

  ledcSetup(coolPwmChannel, pwmFrequency,
            pwmResolution); // Canal 1 para Enfriar (Pin 27)
  ledcAttachPin(PWM_COOLING_PIN, coolPwmChannel);

  pinMode(EN_HEATER_PIN, OUTPUT);
  pinMode(EN_COOLING_PIN, OUTPUT);

  stop();
  logln("Heater/Cooler initialized with PWM Dual (5kHz).");
}

void Heater::setPower(float dutyCycle, bool fineZone) {
  dutyCycle = constrain(dutyCycle, -100.0f, 100.0f);

  // Extraer magnitud y signo
  float magnitude = fabs(dutyCycle);
  float sign = (dutyCycle >= 0) ? 1.0f : -1.0f;

  // --------------------------------------------------
  // Micro-PWM térmico metrológico (zona fina simétrica)
  // --------------------------------------------------
  if (fineZone && magnitude > 0.0f && magnitude < microFine.minPower) {

    static uint32_t t0 = 0;
    uint32_t now = millis();

    if (t0 == 0)
      t0 = now;

    uint32_t phase = (now - t0) % microFine.periodMs;

    // Calculamos qué potencia usar:
    // Si el PID pide más que el mínimo, usamos lo que pide el PID como "Pico".
    // Si pide menos, usamos el minPower para asegurar que la resistencia
    // caliente.
    float peakPower =
        (magnitude > microFine.minPower) ? magnitude : microFine.minPower;

    if (phase < microFine.onTimeMs) {
      dutyCycle = peakPower * sign;
    } else {
      dutyCycle = 0.0f;
    }
  } else {
    // Si salimos de la zona fina, podrías resetear t0 para el próximo ciclo
    // t0 = 0;
  }

  // --------------------------------------------------
  // Actuación final (Despacho de potencia)
  // --------------------------------------------------
  if (dutyCycle > 0.0f) {
    setHeat(dutyCycle);
  } else if (dutyCycle < 0.0f) {
    setCool(fabs(dutyCycle));
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
  dutyCycle = constrain(dutyCycle, 0.0f, 100.0f);

  uint32_t maxDuty = (1 << pwmResolution) - 1;

  uint32_t duty = 0;

  if (dutyCycle > 0.0f) {
    float linear_duty = (dutyCycle / 100.0f) * (float)maxDuty;
    duty = (uint32_t)linear_duty;
  }

  if (duty > maxDuty)
    duty = maxDuty; // Cap final (solo por seguridad)

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
  dutyCycle = constrain(dutyCycle, 0.0f, 100.0f);

  uint32_t maxDuty = (1 << pwmResolution) - 1;

  uint32_t duty = 0;

  if (dutyCycle > 0.0f) {
    float linear_duty = (dutyCycle / 100.0f) * (float)maxDuty;
    duty = (uint32_t)linear_duty;
  }

  if (duty > maxDuty)
    duty = maxDuty;

  digitalWrite(EN_HEATER_PIN, HIGH);
  digitalWrite(EN_COOLING_PIN, HIGH);

  // MUTUA EXCLUSIÓN: Asegurar que el lado de Calentamiento esté en 0
  ledcWrite(heatPwmChannel, 0);

  if (duty > 0)
    logf("Cooling set to %.2f%% power (Duty: %u)\n", dutyCycle, duty);

  // Aplicar PWM al lado de Enfriamiento
  ledcWrite(coolPwmChannel, duty);
}

void Heater::stop() {
  ledcWrite(heatPwmChannel, 0);
  ledcWrite(coolPwmChannel, 0);

  digitalWrite(EN_HEATER_PIN, LOW);
  digitalWrite(EN_COOLING_PIN, LOW);

  logln("Heater/Cooler stopped.");
}

// void Heater::hold(float temp, float setpoint) {
//   static float lastTemp = temp;
//   static uint32_t lastPulseTime = 0;

//   uint32_t now = millis();
//   float dt = (now - lastPulseTime) / 1000.0f;
//   if (dt <= 0)
//     dt = 0.1f;

//   float dTdt = (temp - lastTemp) / dt;
//   lastTemp = temp;

//   // --- Parámetros de HOLD ---
//   // const float DEAD = 0.3f;                  // ya estás dentro
//   const float COOL_RATE_TH = -0.005f;       // °C/s
//   const uint32_t MIN_PULSE_INTERVAL = 2000; // ms
//   const float HOLD_PWM = 3.0f;              // % muy suave

//   // 1️⃣ Nunca actuar si aún sube
//   if (dTdt >= 0.0f) {
//     // logln("Hold: Temp rising or stable. No action taken.");
//     setPower(0.0f, true); // sin energía
//     return;
//   }

//   // 2️⃣ Si se enfría naturalmente
//   if (dTdt < COOL_RATE_TH) {
//     // 3️⃣ Respetar intervalo entre pulsos
//     if (now - lastPulseTime > MIN_PULSE_INTERVAL) {
//       logf("Hold: Temp dropping (%.4f °C/s). Applying pulse.\n", dTdt);
//       setHeat(HOLD_PWM);
//       delay(200); // pulso corto
//       setPower(0.0f, true);
//       lastPulseTime = now;
//     }
//   }
// }

void Heater::hold(float temp, float setpoint) {
  static float lastTemp = temp;
  static uint32_t lastPulseTime = 0;
  static uint32_t lastSampleTime = 0;

  uint32_t now = millis();
  float dt = (now - lastSampleTime) / 1000.0f;

  // Protección contra llamadas demasiado frecuentes (evita dt = 0)
  if (dt < 0.1f)
    return;

  float dTdt = (temp - lastTemp) / dt;

  // --- PARÁMETROS CRÍTICOS ---
  const float HOLD_PWM = 5.0f;
  // 1. Bajamos el umbral: el sensor baja de 0.01 en 0.01.
  // Con -0.005 capturamos cualquier bajada real.
  const float COOL_RATE_TH = -0.005f;

  // --- LÓGICA DE ACTIVACIÓN ---

  // 1️⃣ REGLA DE ORO: Si estamos por encima del setpoint, NUNCA calentar.
  // El modo HOLD es para evitar que caiga, no para alimentar un overshoot.
  if (temp > setpoint) {
    this->stop(); // Asegurar apagado
    lastTemp = temp;
    lastSampleTime = now;
    return;
  }

  // 2️⃣ Solo actuar si estamos en la "zona de peligro" (bajo el setpoint)
  // pero no tan abajo como para que el PID deba tomar el control total.
  else if (temp <= (setpoint + 0.0f) && temp >= (setpoint - 0.5f)) {
    if (dTdt < COOL_RATE_TH) {
      if (now - lastPulseTime >= 1000) {
        this->setHeat(HOLD_PWM);
        lastPulseTime = now;
        logf("!!! HOLD PULSE !!! Preventivo. Temp: %.2f\n", temp);
      }
    }
  } else {
    // Fuera de rango: dejamos que el PID actúe normalmente
    this->stop(); // Asegurar apagado
  }

  // Actualizamos para la siguiente iteración
  lastTemp = temp;
  lastSampleTime = now;
}