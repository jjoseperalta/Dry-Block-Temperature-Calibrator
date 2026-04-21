#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Settings.h"
#include <Arduino.h>
#include <QuickPID.h>

// Estados térmicos del sistema
enum class ThermalState {
  RAMPING,     // Lejos del setpoint -> tunings agresivos
  APPROACHING, // Cerca -> transición/interpolación y frenado suave
  HOLDING      // Muy cerca -> tunings de precisión estables
};

class PIDController {
public:
  PIDController(Settings &settings);

  // Calcula la salida PID basada en setpoint, temperatura actual y dt
  float compute(float setpoint, float input, float dt, bool forceCoolingBrake = false);

  // Reinicia el estado interno del PID y sus acumuladores
  void reset();

  // Establece la temperatura anterior (útil para mantener compatibilidad)
  void setPreviousPV(float previousPV);

  // Getters/Setters para tuning manual
  void setTunings(float kp, float ki, float kd);
  void getTunings(float &kp, float &ki, float &kd);
  void printCurrentTunings();

  // Configuración de límites
  void setOutputLimits(float min, float max);

  ThermalState getCurrentState() const { return currentState; }
  const char *getStateString() const;

private:
  Settings &settings;

  // Variables para QuickPID
  float currentInput;    
  float currentOutput;   
  float currentSetpoint; 

  // Variables de estado y rampas
  float lastOutput;
  float previousPV;
  bool firstRun;

  // Instancia de QuickPID
  QuickPID myPID;

  // =========================================================================
  // PARÁMETROS DEL PID (Sintonización por defecto)
  // =========================================================================
  
  // Parámetros de enfriamiento (¡Mantener, son tu excelente curva!)
  static constexpr float DEFAULT_KP_COOL = 1.4f;
  static constexpr float DEFAULT_KI_COOL = 0.05f;
  static constexpr float DEFAULT_KD_COOL = 4.0f;

  // Enfriamiento agresivo (¡Mantener!)
  static constexpr float DEFAULT_AGGR_KP_COOL = 4.2f;
  static constexpr float DEFAULT_AGGR_KI_COOL = 0.02f;
  static constexpr float DEFAULT_AGGR_KD_COOL = 1.2f;

  // =========================================================================
  // NUEVOS AJUSTES: Dinámica balanceada para calentamiento sin estancamiento
  // =========================================================================

  // Parámetros de calentamiento (Modo APPROACHING/HOLDING)
  static constexpr float DEFAULT_KP_HEAT = 1.8f;   // Sube de 0.8f a 1.2f para mayor firmeza en la aproximación
  static constexpr float DEFAULT_KI_HEAT = 0.08f;  // Sube de 0.02f a 0.05f para absorber el error remanente rápidamente
  static constexpr float DEFAULT_KD_HEAT = 3.5f;   // Baja de 8.5f a 6.5f para que no sobre-amortigüe el tramo final

  // Tunings agresivos de calentamiento (Modo RAMPING lejano)
  static constexpr float DEFAULT_AGGR_KP_HEAT = 3.5f;  
  static constexpr float DEFAULT_AGGR_KI_HEAT = 0.04f;  
  static constexpr float DEFAULT_AGGR_KD_HEAT = 1.5f;

  // Parámetros de frenado por cercanía (Multiplicadores de fuerza)
  static constexpr float COAST_DISTANCE_HEAT = 4.0f; 
  static constexpr float COAST_DISTANCE_COOL = 4.0f; 

  // Limitaciones y dinámicas
  static constexpr float MAX_OUTPUT_CHANGE = 25.0f; // Sube de 10 a 35 para dar agilidad al lazo
  static constexpr float OUTPUT_MIN = -100.0f;       
  static constexpr float OUTPUT_MAX = 100.0f;        

  // Umbrales de estados térmicos
  // Invertimos la relación: le damos más rango dinámico al frío porque le cuesta más avanzar
  static constexpr float RAMPING_THRESHOLD_HEAT = 12.0f;
  static constexpr float RAMPING_THRESHOLD_COOL = 8.0f; // Mantiene potencia alta hasta estar a 12°C del setpoint
  static constexpr float HOLDING_THRESHOLD = 0.35f;      // Ajustado a la resolución típica

  // Métodos privados de control adaptativo
  ThermalState determineThermalState(float absError, bool isHeating);
  void applyAdaptiveTunings(ThermalState state, bool isHeating, float absError);
  float applyBraking(float output, float absError, bool isHeating, bool forceCoolingBrake);
  float applySlewRate(float output, bool isHeating);
  void logTuningChange(ThermalState newState, bool isHeating, float newKp, float newKi, float newKd);

  ThermalState currentState;
  ThermalState lastLoggedState;
  bool lastLoggedHeating;
  float lastLoggedKp;
  float lastLoggedKi;
  float lastLoggedKd;
};

#endif // PIDCONTROLLER_H