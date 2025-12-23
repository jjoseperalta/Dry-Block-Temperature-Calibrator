#include "Buzzer.h"
#include "Calibration.h"
#include "HMIController.h"
#include "Heater.h"
#include "Logger.h"
#include "PIDController.h"
#include "Sensors.h"
#include "Settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <cmath>

#define CMD_BUFFER_SIZE 64

// Function prototypes
void handleCommand(const char *command);
void printHelp();
void printSettings();
void printTemperatures();
void renderNavBarUI();
void renderConfigUI(u_int8_t page);
bool isStableEnoughForCalibration(float masterTemp, float setpoint);

// Prototipos de las tareas de FreeRTOS
void taskControlCore(void *parameter);
void taskInterfaceCore(void *parameter);
void taskConsole(void *parameter);

// Global objects
Settings settings;
Sensors sensors(settings);
Heater heater;
Buzzer buzzer;
PIDController pid(settings);
Calibration calibration(settings, sensors, buzzer);
HMIController hmi;

enum class ThermalState { RAMPING, APPROACHING, HOLDING };
static ThermalState thermalState = ThermalState::RAMPING;
const char *stateNames[] = {"RAMPING", "APPROACHING", "HOLDING"};
enum class ControlState { STOPPED, RUNNING, IDLE };
static ControlState controlState = ControlState::IDLE;

unsigned long lastPidTime = 0;
bool targetReachedNotified = false;

const long intervalo = 500;
unsigned long tiempoAnterior = 0;

char buffer[32];
size_t buffer_size = sizeof(buffer);

float masterTemp = 0.0f;

// VARIABLES Y OBJETOS DE FREERTOS
SemaphoreHandle_t controlStateMutex; // Mutex para proteger controlState
SemaphoreHandle_t masterTempMutex;   // Mutex para proteger masterTemp
SemaphoreHandle_t serialMutex;       // Mutex para proteger acceso a Serial

void callbackSensorType(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Sensor Type triggered");
  if (type == NEX_EVENT_POP) {
    NextionDualStateButton *btn = static_cast<NextionDualStateButton *>(widget);

    btn->isActive() ? settings.setSensorType(SensorType::PT1000)
                    : settings.setSensorType(SensorType::PT100);

    logf("Sensor type set to: %s\n",
         settings.getSensorType() == SensorType::PT1000 ? "PT1000" : "PT100");

    settings.save();
  }
}

void callbackTemperatureScale(NextionEventType type,
                              INextionTouchable *widget) {
  // logln("Callback Temperature Scale triggered");
  if (type == NEX_EVENT_POP) {
    NextionDualStateButton *btn = static_cast<NextionDualStateButton *>(widget);

    btn->isActive() ? settings.setTemperatureScale(TemperatureScale::FAHRENHEIT)
                    : settings.setTemperatureScale(TemperatureScale::CELSIUS);

    logf("Temperature scale set to: %s\n",
         settings.getTemperatureScale() == TemperatureScale::FAHRENHEIT
             ? "Fahrenheit"
             : "Celsius");

    settings.save();
  }
}

void callbackPage0(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Page 0 triggered");
  if (type == NEX_EVENT_PUSH) {
    pt_0.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_0.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    // pt_0.setActive(settings.getSensorType() == SensorType::PT1000);
    // scale_0.setActive(settings.getTemperatureScale() ==
    //                   TemperatureScale::FAHRENHEIT);
  }
}

void callbackPage1(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Page 1 triggered");
  if (type == NEX_EVENT_PUSH) {
    pt_1.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_1.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    // pt_1.setActive(settings.getSensorType() == SensorType::PT1000);
    // scale_1.setActive(settings.getTemperatureScale() ==
    //                   TemperatureScale::FAHRENHEIT);

    char buffer[16];
    sprintf(buffer, "%.2f", settings.getSetTemperature());
    setp.setText(buffer);
    // setp.setText(buffer);

    logf("SETP displayed as: %s\n", buffer);
  }
}

void callbackPage2(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Page 2 triggered");
  if (type == NEX_EVENT_PUSH) {
    pt_2.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_2.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    // pt_2.setActive(settings.getSensorType() == SensorType::PT1000);
    // scale_2.setActive(settings.getTemperatureScale() ==
    //                   TemperatureScale::FAHRENHEIT);
  }
}

void callbackPage3(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Page 3 triggered");
  if (type == NEX_EVENT_PUSH) {
    pt_3.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_3.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    // pt_3.setActive(settings.getSensorType() == SensorType::PT1000);
    // scale_3.setActive(settings.getTemperatureScale() ==
    //                   TemperatureScale::FAHRENHEIT);
    renderConfigUI(3);
    // renderConfigUI(3);
  }
}

void callbackPage4(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Page 4 triggered");
  if (type == NEX_EVENT_PUSH) {
    pt_4.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_4.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    // pt_4.setActive(settings.getSensorType() == SensorType::PT1000);
    // scale_4.setActive(settings.getTemperatureScale() ==
    //                   TemperatureScale::FAHRENHEIT);
    renderConfigUI(4);
    // renderConfigUI(4);
  }
}

void callbackPage5(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Page 5 triggered");
  if (type == NEX_EVENT_PUSH) {
    pt_5.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_5.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    // pt_5.setActive(settings.getSensorType() == SensorType::PT1000);
    // scale_5.setActive(settings.getTemperatureScale() ==
    //                   TemperatureScale::FAHRENHEIT);
    renderConfigUI(5);
    // renderConfigUI(5);
  }
}

void callbackQuickTest(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Quick Test triggered");
  if (type == NEX_EVENT_POP) {
    u_int8_t indexBtn = widget->getComponentID();

    if (indexBtn == heat.getComponentID() ||
        indexBtn == cool.getComponentID()) {

      settings.setSetTemperature(
          setp.getText(buffer, buffer_size) ? atof(buffer) : 0.0f);

      logf("SETP set to: %.2f\n", settings.getSetTemperature());

      controlState = ControlState::RUNNING;

      pid.reset();

      pid.setPreviousPV(masterTemp);

      logf("PID control started. Target: %.2f\n", settings.getSetTemperature());

    } else if (indexBtn == stop_1.getComponentID()) {
      controlState = ControlState::STOPPED;

      heater.stop();

      calibration.stop();

      logln("All processes stopped.");
    }
  }
}

void callbackAdvancedTest(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Advanced Test triggered");
  if (type == NEX_EVENT_POP) {
    u_int8_t indexBtn = widget->getComponentID();
    // logf("Button ID: %d\n", indexBtn);
    if (indexBtn == run.getComponentID()) {
      controlState = ControlState::RUNNING;
      pid.reset();
      pid.setPreviousPV(masterTemp);
      calibration.start();
      logln("Calibration started.");
    } else if (indexBtn == stop_2.getComponentID()) {
      controlState = ControlState::STOPPED;
      heater.stop();
      calibration.stop();
      logln("All processes stopped.");
    }
  }
}

void callbackConfig(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Config triggered");
  if (type == NEX_EVENT_POP) {
    u_int8_t indexBtn = widget->getComponentID();

    if (indexBtn == default_3.getComponentID() ||
        indexBtn == default_4.getComponentID() ||
        indexBtn == default_5.getComponentID()) {

      settings.resetToDefaults();

      // renderConfigUI();

      logln("Settings reset to default values.");

    } else if (indexBtn == save_3.getComponentID() ||
               indexBtn == save_4.getComponentID() ||
               indexBtn == save_5.getComponentID()) {

      char buffer[16];

      kp.getText(buffer, sizeof(buffer));
      settings.setPidKp((buffer[0] != '\0') ? atof(buffer)
                                            : Settings::getDefaultPidkp());
      ti.getText(buffer, sizeof(buffer));
      settings.setPidTi((buffer[0] != '\0') ? atof(buffer)
                                            : Settings::getDefaultPidTi());
      td.getText(buffer, sizeof(buffer));
      settings.setPidTd((buffer[0] != '\0') ? atof(buffer)
                                            : Settings::getDefaultPidTd());
      period.getText(buffer, sizeof(buffer));
      settings.setPidPeriod(
          (buffer[0] != '\0') ? atoi(buffer) : Settings::getDefaultPidPeriod());
      stability.getText(buffer, sizeof(buffer));
      settings.setStabilityTime((buffer[0] != '\0')
                                    ? atoi(buffer)
                                    : Settings::getDefaultStabilityTime());
      setp1.getText(buffer, sizeof(buffer));
      settings.setCalibrationPoint(
          0, (buffer[0] != '\0') ? atof(buffer) : Settings::getDefaultP1());
      setp2.getText(buffer, sizeof(buffer));
      settings.setCalibrationPoint(
          1, (buffer[0] != '\0') ? atof(buffer) : Settings::getDefaultP2());
      setp3.getText(buffer, sizeof(buffer));
      settings.setCalibrationPoint(
          2, (buffer[0] != '\0') ? atof(buffer) : Settings::getDefaultP3());
      // setp4.getText(buffer, sizeof(buffer));
      // settings.setCalibrationPoint(
      //     3, (buffer[0] != '\0') ? atof(buffer) : Settings::getDefaultP4());
      upperlimit.getText(buffer, sizeof(buffer));
      settings.setAlarmUpperLimit((buffer[0] != '\0')
                                      ? atof(buffer)
                                      : Settings::getDefaultAlarmUpperLimit());
      lowerlimit.getText(buffer, sizeof(buffer));
      settings.setAlarmLowerLimit((buffer[0] != '\0')
                                      ? atof(buffer)
                                      : Settings::getDefaultAlarmLowerLimit());
      danger.getText(buffer, sizeof(buffer));
      settings.setDangerTemperature(
          (buffer[0] != '\0') ? atof(buffer)
                              : Settings::getDefaultDangerTemperature());
      safe.getText(buffer, sizeof(buffer));
      settings.setSafeTemperature((buffer[0] != '\0')
                                      ? atof(buffer)
                                      : Settings::getDefaultSafeTemperature());

      settings.save();

      logln("Settings saved.");
    }
  }
}

void handleCalibrationUpdate(int index) {
  logf("MAIN: Callback recibido. El punto %i fue registrado.", index);

  const CalibrationData &pointData = calibration.getCalibrationData(index);

  // 2. Mostrar los valores en la HMI
  log("SETP: ");
  logf("%.2f\n", pointData.setpoint);
  log("MASTER: ");
  logf("%.2f\n", pointData.masterTemp);
  log("TEST: ");
  logf("%.2f\n", pointData.testTemp);
  log("DIFF: ");
  logf("%.3f\n", pointData.difference);

  if (index == 0) {
    snprintf(buffer, buffer_size, "%.1f", pointData.setpoint);
    temp_p1.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", pointData.masterTemp);
    up_temp_master_p1.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", pointData.testTemp);
    up_temp_test_p1.setText(buffer);
    snprintf(buffer, buffer_size, "%.3f", pointData.difference);
    up_diff_p1.setText(buffer);
  } else if (index == 1) {
    snprintf(buffer, buffer_size, "%.1f", pointData.setpoint);
    temp_p2.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", pointData.masterTemp);
    up_temp_master_p2.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", pointData.testTemp);
    up_temp_test_p2.setText(buffer);
    snprintf(buffer, buffer_size, "%.3f", pointData.difference);
    up_diff_p2.setText(buffer);
  } else if (index == 2) {
    snprintf(buffer, buffer_size, "%.1f", pointData.setpoint);
    temp_p3.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", pointData.masterTemp);
    up_temp_master_p3.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", pointData.testTemp);
    up_temp_test_p3.setText(buffer);
    snprintf(buffer, buffer_size, "%.3f", pointData.difference);
    up_diff_p3.setText(buffer);
  } else if (index == 3) {
    snprintf(buffer, buffer_size, "%.1f", pointData.setpoint);
    temp_p4.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", pointData.masterTemp);
    up_temp_master_p4.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", pointData.testTemp);
    up_temp_test_p4.setText(buffer);
    snprintf(buffer, buffer_size, "%.3f", pointData.difference);
    up_diff_p4.setText(buffer);
  } else if (index == 4) {
    snprintf(buffer, buffer_size, "%.1f", pointData.setpoint);
    down_temp_master_p4.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", pointData.testTemp);
    down_temp_test_p4.setText(buffer);
    snprintf(buffer, buffer_size, "%.3f", pointData.difference);
    down_diff_p4.setText(buffer);
  } else if (index == 5) {
    snprintf(buffer, buffer_size, "%.1f", pointData.setpoint);
    down_temp_master_p3.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", pointData.testTemp);
    down_temp_test_p3.setText(buffer);
    snprintf(buffer, buffer_size, "%.3f", pointData.difference);
    down_diff_p3.setText(buffer);
  } else if (index == 6) {
    snprintf(buffer, buffer_size, "%.1f", pointData.setpoint);
    down_temp_master_p2.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", pointData.testTemp);
    down_temp_test_p2.setText(buffer);
    snprintf(buffer, buffer_size, "%.3f", pointData.difference);
    down_diff_p2.setText(buffer);
  } else if (index == 7) {
    snprintf(buffer, buffer_size, "%.1f", pointData.setpoint);
    down_temp_master_p1.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", pointData.testTemp);
    down_temp_test_p1.setText(buffer);
    snprintf(buffer, buffer_size, "%.3f", pointData.difference);
    down_diff_p1.setText(buffer);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for serial port to connect.
  }

  // Llama a Settings::begin(), que monta LittleFS y llama a settings.load()
  settings.begin();
  sensors.begin();
  heater.begin();
  buzzer.begin();
  hmi.init();

  buzzer.playBlocking(BeepType::READY);

  calibration.setRegisterCallback(handleCalibrationUpdate);

  logln("Dry Block Temperature Calibrator");
  logln("Type 'HELP' for a list of commands.");

  // DEBUG: Imprimimos la configuración inmediatamente después de cargar
  logln("--- DEBUG: Configuración cargada ---");
  printSettings();
  logln("------------------------------------");

  // Initialize PID timer
  lastPidTime = millis();

  // Register callback
  HMIController::registerCallback(&pt_0, callbackSensorType);
  HMIController::registerCallback(&scale_0, callbackTemperatureScale);
  HMIController::registerCallback(&quickTest, callbackPage1);
  HMIController::registerCallback(&advancedTest, callbackPage2);

  HMIController::registerCallback(&pt_1, callbackSensorType);
  HMIController::registerCallback(&scale_1, callbackTemperatureScale);
  HMIController::registerCallback(&heat, callbackQuickTest);
  HMIController::registerCallback(&cool, callbackQuickTest);
  HMIController::registerCallback(&stop_1, callbackQuickTest);
  HMIController::registerCallback(&home_1, callbackPage0);

  HMIController::registerCallback(&pt_2, callbackSensorType);
  HMIController::registerCallback(&scale_2, callbackTemperatureScale);
  HMIController::registerCallback(&run, callbackAdvancedTest);
  HMIController::registerCallback(&config, callbackPage3);
  HMIController::registerCallback(&stop_2, callbackAdvancedTest);
  HMIController::registerCallback(&home_2, callbackPage0);

  HMIController::registerCallback(&pt_3, callbackSensorType);
  HMIController::registerCallback(&scale_3, callbackTemperatureScale);
  HMIController::registerCallback(&next_3, callbackPage4);
  HMIController::registerCallback(&default_3, callbackConfig);
  HMIController::registerCallback(&save_3, callbackConfig);
  HMIController::registerCallback(&backAdvancedTest_3, callbackPage2);

  HMIController::registerCallback(&pt_4, callbackSensorType);
  HMIController::registerCallback(&scale_4, callbackTemperatureScale);
  HMIController::registerCallback(&back_4, callbackPage3);
  HMIController::registerCallback(&next_4, callbackPage5);
  HMIController::registerCallback(&default_4, callbackConfig);
  HMIController::registerCallback(&save_4, callbackConfig);
  HMIController::registerCallback(&backAdvancedTest_4, callbackPage2);

  HMIController::registerCallback(&pt_5, callbackSensorType);
  HMIController::registerCallback(&scale_5, callbackTemperatureScale);
  HMIController::registerCallback(&back_5, callbackPage4);
  HMIController::registerCallback(&default_5, callbackConfig);
  HMIController::registerCallback(&save_5, callbackConfig);
  HMIController::registerCallback(&backAdvancedTest_5, callbackPage2);

  controlStateMutex = xSemaphoreCreateMutex();
  masterTempMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();

  // Tarea 1: Control (Core 1 / App Core) - Mayor prioridad para el control
  xTaskCreatePinnedToCore(taskControlCore,          // Función a ejecutar
                          "ControlTask",            // Nombre de la tarea
                          10000,                    // Tamaño de pila (bytes)
                          NULL,                     // Parámetro de la tarea
                          configMAX_PRIORITIES - 1, // Prioridad (Alta)
                          NULL, // Handle de la tarea (No necesario aquí)
                          1     // NÚCLEO 1 (App Core)
  );

  // Tarea 2: Interfaz (Core 0 / Pro Core) - Baja prioridad, permite al SO
  // funcionar
  xTaskCreatePinnedToCore(taskInterfaceCore, // Función a ejecutar
                          "InterfaceTask",   // Nombre de la tarea
                          4000,              // Tamaño de pila (bytes)
                          NULL,              // Parámetro de la tarea
                          1,                 // Prioridad (Baja)
                          NULL,              // Handle de la tarea
                          0                  // NÚCLEO 0 (Pro Core)
  );

  xTaskCreatePinnedToCore(taskConsole, "ConsoleTask", 4096, nullptr,
                          1, // prioridad baja
                          nullptr,
                          0 // Core 0 (interfaz)
  );
}

void loop() {}

// Tarea 1: Control y Sensores (Core 1)
void taskControlCore(void *parameter) {
  // Variables de control de estado y tiempo
  static uint32_t stableTime = 0;
  static float stableSum = 0.0f;
  static uint32_t stableSamples = 0;
  static bool wasHolding = false;
  static float lastTempForHold = NAN;
  static float tempDeltaForHold = 0.0f;
  static ControlState lastState = ControlState::STOPPED;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    Serial.flush();

    // Cálculo de Frecuencia (Sincronizado con settings)
    TickType_t xPidFrequency = pdMS_TO_TICKS(settings.getPidPeriod() * 1000UL);
    if (xPidFrequency == 0)
      xPidFrequency = pdMS_TO_TICKS(100UL);

    // Obtener Lectura de Sensor
    float currentMasterTemp = sensors.getFilteredMasterTemperature(0.1f);
    float currentTestTemp = sensors.getFilteredTestTemperature(0.1f);

    // Comunicación Inter-Core: Escritura de masterTemp (Protegida)
    if (xSemaphoreTake(masterTempMutex, (TickType_t)10) == pdTRUE) {
      masterTemp = currentMasterTemp;
      xSemaphoreGive(masterTempMutex);
    }

    // --- Lógica de Estados Térmicos ---
    float setpoint = settings.getSetTemperature();
    float error = setpoint - currentMasterTemp;

    // Implementa una pequeña histéresis para evitar el "titubeo" de estados
    if (thermalState == ThermalState::RAMPING) {
      if (error < 0.8f)
        thermalState =
            ThermalState::APPROACHING; // No cambia hasta bajar de 0.8
    } else if (thermalState == ThermalState::APPROACHING) {
      if (error > 1.2f)
        thermalState =
            ThermalState::RAMPING; // No vuelve a RAMPING hasta subir de 1.2
      if (error < 0.10f)
        thermalState = ThermalState::HOLDING;
    } else if (thermalState == ThermalState::HOLDING) {
      if (error > 0.5f)
        thermalState = ThermalState::APPROACHING;
    }

    // Obtener Estado de Control (Mutex)
    ControlState currentState;
    if (xSemaphoreTake(controlStateMutex, (TickType_t)10) == pdTRUE) {
      currentState = controlState;
      xSemaphoreGive(controlStateMutex);
    } else {
      currentState = ControlState::STOPPED;
    }

    if (currentState != lastState) {
      if (currentState == ControlState::STOPPED) {
        heater.stop(); // Se llama SOLO al entrar en STOPPED
        stableTime = 0;
        stableSum = 0.0f;
        stableSamples = 0;
        targetReachedNotified = false;
        logln("System STOPPED: Heater off.");
      }
      lastState = currentState;
    }

    // Ejecución del PID o Mantenimiento
    if (currentState == ControlState::RUNNING) {
      float dt = xPidFrequency / (float)configTICK_RATE_HZ;
      if (dt <= 0)
        dt = 1.0f;

      static float lastTemp = currentMasterTemp;
      float dTdt = (currentMasterTemp - lastTemp) / dt;
      lastTemp = currentMasterTemp;

      float output = pid.calculate(setpoint, currentMasterTemp, dt);

      static float frenoExtra = 0.0f;
      static uint32_t ultimoPulsoFreno = 0;

      logf("State: %s | Error: %.2f | Output: %.2f | Temp: %.2f | Setpoint: "
           "%.2f\n",
           stateNames[(int)thermalState], error, output, currentMasterTemp,
           setpoint);

      switch (thermalState) {
      case ThermalState::RAMPING:
        heater.setPower(output, false);
        break;
      case ThermalState::APPROACHING:
      case ThermalState::HOLDING:
        if (millis() - ultimoPulsoFreno > 3000) {
          frenoExtra = 0.0f;
        }

        if (dTdt < -0.005f && currentMasterTemp < (setpoint + 0.15f)) {
          if (millis() - ultimoPulsoFreno > 3000) {
            frenoExtra = fabs(dTdt) * 35.0f + 2.0f;
            ultimoPulsoFreno = millis();
            logf("!!! FRENO INTELIGENTE !!! Potencia: +%.1f%%\n", frenoExtra);
          }
        }
        float potenciaFinal = output + frenoExtra;

        if (potenciaFinal < 0 && currentMasterTemp < (setpoint + 0.05f)) {
          potenciaFinal = 0.0f;
        }
        heater.setPower(potenciaFinal, true);
        break;
      }

      // Lógica de Estabilidad (Alarma Reached)
      float tolerance =
          settings.getCalibrationTolerance();

      if (currentMasterTemp >= (setpoint - 0.01f) &&
          currentMasterTemp <= (setpoint + 0.1f)) {
        stableSamples++;
        stableSum += currentMasterTemp;

        // Sumamos el tiempo de este ciclo
        stableTime += settings.getPidPeriod();

        if (stableTime >= settings.getStabilityTime()) {
          float avg = stableSum / (float)stableSamples;
          // Verificamos si el PROMEDIO también está dentro de la tolerancia
          if (fabs(avg - setpoint) <= tolerance) {
            if (!targetReachedNotified) {
              buzzer.beep(BeepType::TARGET_REACHED);
              targetReachedNotified = true;
              logln("Thermal stability reached (within tolerance).");
              if (calibration.isRunning())
                calibration.notifyStable(currentMasterTemp, currentTestTemp);
            }
          }
        }
      } else {
        stableTime = 0;
        stableSum = 0.0f;
        stableSamples = 0;
        targetReachedNotified = false;
      }
    }

    // Alarmas Críticas (Siempre activas)
    if (currentMasterTemp > settings.getAlarmUpperLimit() ||
        currentMasterTemp < settings.getAlarmLowerLimit()) {
      buzzer.beep(BeepType::ALARM);
    }

    // Sincronización de tarea
    vTaskDelayUntil(&xLastWakeTime, xPidFrequency);
  }
}

// Tarea 2: Interfaz HMI y Comunicación Serial (Core 0)
void taskInterfaceCore(void *parameter) {
  // --- Configuración de Frecuencias ---

  // Frecuencia RÁPIDA (Base de la tarea): 10ms
  const TickType_t xFastPollFrequency = pdMS_TO_TICKS(10);
  // Frecuencia LENTA (Actualización de UI): 500ms / 10ms = 50 ciclos
  const int UPDATE_CYCLES = 50;

  // Variable para rastrear el tiempo del último despertar (CRÍTICA para
  // DelayUntil)
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Contador de ciclos para el temporizador de 500ms
  int updateCycleCounter = 0;

  while (true) {
    // ***************************************************************
    // 1. TAREAS DE ALTA FRECUENCIA (Ejecutadas en cada ciclo de 10ms)
    // ***************************************************************
    buzzer.handle();
    hmi.poll(); // ¡CRÍTICO! Polling de la HMI

    // --- COMUNICACIÓN SERIAL (Comandos) ---
    // NOTA: Es seguro usar String aquí si el comando serial es infrecuente
    // y se usa readStringUntil, pero se recomienda Char Array para máxima
    // robustez.
    // if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    //   if (Serial.available() > 0) {
    //     String command = Serial.readStringUntil('\n');
    //     command.trim();
    //     command.toUpperCase();
    //     handleCommand(command);
    //   }
    //   xSemaphoreGive(serialMutex);
    // }

    // ***************************************************************
    // 2. ACTUALIZACIÓN DE UI (Lógica de 500ms / 50 ciclos)
    // ***************************************************************
    updateCycleCounter++;
    if (updateCycleCounter >= UPDATE_CYCLES) {
      updateCycleCounter = 0; // Resetear el contador de ciclos

      // --- LECTURA PROTEGIDA DE TEMPERATURA ---
      float tempToDisplay = 0.0f;
      // Lectura del valor compartido (MasterTemp)
      if (xSemaphoreTake(masterTempMutex, (TickType_t)10) == pdTRUE) {
        tempToDisplay = masterTemp;
        xSemaphoreGive(masterTempMutex);
      }

      // --- FORMATO Y ESCRITURA DE TEXTO HMI ---

      // Obtener la unidad de temperatura (Uso de 'const char *' es correcto)
      const char *unit =
          settings.getTemperatureScale() == TemperatureScale::CELSIUS ? "C"
                                                                      : "F";

      // Actualizar Widget temp_1 (Temperatura actual con unidad)
      // snprintf es el método correcto para formateo seguro en
      // microcontroladores
      snprintf(buffer, buffer_size, "%.2f \xB0%s", tempToDisplay, unit);
      temp_1.setText(buffer);

      // Actualizar Widget temp_2 (Temperatura actual / Setpoint)
      snprintf(buffer, buffer_size, "%.2f/%.2f", tempToDisplay,
               settings.getSetTemperature());
      temp_2.setText(buffer);
    }

    // ***************************************************************
    // 3. CEDER TIEMPO (Garantizar la cadencia rápida de 10ms)
    // ***************************************************************
    // Usar vTaskDelayUntil garantiza que la tarea se despierte exactamente
    // 10ms después de su último despertar, previniendo el 'time drift'.
    vTaskDelayUntil(&xLastWakeTime, xFastPollFrequency);
  }
}

void taskConsole(void *parameter) {
  static char cmdBuffer[CMD_BUFFER_SIZE];
  size_t idx = 0;

  for (;;) {
    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (idx > 0) {
          cmdBuffer[idx] = '\0';
          handleCommand(cmdBuffer); // 👈 ahora char*
          idx = 0;
        }
      } else if (idx < CMD_BUFFER_SIZE - 1) {
        cmdBuffer[idx++] = c;
      } else {
        idx = 0; // protección overflow
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void handleCommand(const char *command) {

  // Copia local modificable
  char cmd[CMD_BUFFER_SIZE];
  strncpy(cmd, command, CMD_BUFFER_SIZE);
  cmd[CMD_BUFFER_SIZE - 1] = '\0';

  // Convertir a MAYÚSCULAS
  for (char *p = cmd; *p; ++p) {
    *p = toupper(*p);
  }

  // Tokenización segura
  char *saveptr;
  char *token = strtok_r(cmd, " ", &saveptr);

  if (!token)
    return;

  // =================================================
  // =================== SET =========================
  // =================================================
  if (strcmp(token, "SET") == 0) {

    char *param = strtok_r(nullptr, " ", &saveptr);
    char *value = strtok_r(nullptr, "", &saveptr); // resto de línea

    if (!param || !value) {
      logln("Invalid SET command. Use SET <PARAM> <VALUE>");
      return;
    }

    // -------- PID --------
    if (strcmp(param, "KP") == 0) {
      settings.setPidKp(atof(value));
      logf("KP set to: %.2f\n", settings.getPidKp());

    } else if (strcmp(param, "TI") == 0) {
      settings.setPidTi(atof(value));
      logf("TI set to: %.2f\n", settings.getPidTi());

    } else if (strcmp(param, "TD") == 0) {
      settings.setPidTd(atof(value));
      logf("TD set to: %.2f\n", settings.getPidTd());

    } else if (strcmp(param, "SETP") == 0) {
      settings.setSetTemperature(atof(value));
      logf("SETP set to: %.2f\n", settings.getSetTemperature());

      // -------- OFFSETS --------
    } else if (strcmp(param, "MOFFSET") == 0) {
      settings.setMasterOffset(atof(value));
      logf("MOFFSET set to: %.2f\n", settings.getMasterOffset());

    } else if (strcmp(param, "TOFFSET") == 0) {
      settings.setTestOffset(atof(value));
      logf("TOFFSET set to: %.2f\n", settings.getTestOffset());

      // -------- LIMITES --------
    } else if (strcmp(param, "HIGH") == 0) {
      settings.setAlarmUpperLimit(atof(value));
      logf("Alarm High set to: %.2f\n", settings.getAlarmUpperLimit());

    } else if (strcmp(param, "LOW") == 0) {
      settings.setAlarmLowerLimit(atof(value));
      logf("Alarm Low set to: %.2f\n", settings.getAlarmLowerLimit());

    } else if (strcmp(param, "DANGER") == 0) {
      settings.setDangerTemperature(atof(value));
      logf("Danger Temp set to: %.2f\n", settings.getDangerTemperature());

    } else if (strcmp(param, "SAFE") == 0) {
      settings.setSafeTemperature(atof(value));
      logf("Safe Temp set to: %.2f\n", settings.getSafeTemperature());

      // -------- TIME --------
    } else if (strcmp(param, "PERIOD") == 0) {
      int v = atoi(value);
      if (v > 0) {
        settings.setPidPeriod(v);
        logf("PID Period set to: %d s\n", v);
      } else {
        logln("ERROR: PERIOD must be > 0");
      }

    } else if (strcmp(param, "STABLE") == 0) {
      int v = atoi(value);
      if (v >= 0) {
        settings.setStabilityTime(v);
        logf("Stability Time set to: %d s\n", v);
      } else {
        logln("ERROR: STABLE must be >= 0");
      }

      // -------- CALIB POINTS --------
    } else if (strcmp(param, "P1") == 0) {
      settings.setCalibrationPoint(0, atof(value));
      logf("P1 set to: %.2f\n", settings.getCalibrationPoint(0));

    } else if (strcmp(param, "P2") == 0) {
      settings.setCalibrationPoint(1, atof(value));
      logf("P2 set to: %.2f\n", settings.getCalibrationPoint(1));

    } else if (strcmp(param, "P3") == 0) {
      settings.setCalibrationPoint(2, atof(value));
      logf("P3 set to: %.2f\n", settings.getCalibrationPoint(2));

    } else if (strcmp(param, "P4") == 0) {
      settings.setCalibrationPoint(3, atof(value));
      logf("P4 set to: %.2f\n", settings.getCalibrationPoint(3));

      // -------- ENUMS --------
    } else if (strcmp(param, "SCALE") == 0) {
      if (strcmp(value, "C") == 0) {
        settings.setTemperatureScale(TemperatureScale::CELSIUS);
        logln("Scale set to Celsius");
      } else if (strcmp(value, "F") == 0) {
        settings.setTemperatureScale(TemperatureScale::FAHRENHEIT);
        logln("Scale set to Fahrenheit");
      } else {
        logln("ERROR: SET SCALE C | F");
      }

    } else if (strcmp(param, "SENSOR") == 0) {
      if (strcmp(value, "100") == 0) {
        sensors.configureTestSensor(SensorType::PT100,
                                    settings.getSensorWires());
        logln("Sensor set to PT100");
      } else if (strcmp(value, "1000") == 0) {
        sensors.configureTestSensor(SensorType::PT1000,
                                    settings.getSensorWires());
        logln("Sensor set to PT1000");
      } else {
        logln("ERROR: SET SENSOR 100 | 1000");
      }

    } else if (strcmp(param, "WIRES") == 0) {
      int w = atoi(value);
      if (w >= 2 && w <= 4) {
        sensors.configureTestSensor(settings.getSensorType(), w);
        logf("Sensor wires set to: %d\n", w);
      } else {
        logln("ERROR: WIRES must be 2–4");
      }

    } else {
      logf("Unknown SET parameter: %s\n", param);
    }

    return;
  }

  // =================================================
  // ================= COMMANDS ======================
  // =================================================
  if (strcmp(token, "HEAT") == 0 || strcmp(token, "COOL") == 0) {
    controlState = ControlState::RUNNING;
    pid.reset();
    pid.setPreviousPV(masterTemp);
    logf("PID control started. Target: %.2f\n", settings.getSetTemperature());

  } else if (strcmp(token, "STOP") == 0) {
    controlState = ControlState::STOPPED;
    heater.stop();
    calibration.stop();
    logln("All processes stopped.");

  } else if (strcmp(token, "RUN") == 0) {
    controlState = ControlState::RUNNING;
    pid.reset();
    pid.setPreviousPV(masterTemp);
    calibration.start();
    logln("Calibration started.");

  } else if (strcmp(token, "DEFAULT") == 0) {
    settings.resetToDefaults();
    logln("Defaults restored. Use SAVE.");

  } else if (strcmp(token, "SAVE") == 0) {
    settings.save();
    logln("Settings saved.");

  } else if (strcmp(token, "SHOW") == 0) {
    printSettings();

  } else if (strcmp(token, "TEMP") == 0) {
    printTemperatures();

  } else if (strcmp(token, "HELP") == 0) {
    printHelp();

  } else {
    logf("Unknown command: %s\n", token);
  }
}

void printHelp() {
  logln("Available commands:");
  logln("  TEMP                - Show current block temperatures and "
        "setpoint"); // ¡NUEVO!
  logln("  SET SETP <value>    - Set target temperature");
  logln("  SET KP <value>      - Set Kp for PID");
  logln("  SET TI <value>      - Set Ti for PID (seconds)");
  logln("  SET TD <value>      - Set Td for PID (seconds)");
  logln("  SET PERIOD <value>  - Set PID cycle period (seconds, integer) ");
  logln("  SET STABLE <value>  - Set stability time (seconds, integer)");
  logln("  SET HIGH <value>    - Set high alarm limit");
  logln("  SET LOW <value>     - Set low alarm limit");
  logln("  SET MOFFSET <value> - Set master sensor calibration offset");
  logln("  SET TOFFSET <value> - Set under test sensor calibration offset");
  logln("  SET DANGER <value>  - Set danger temperature limit");
  logln("  SET SAFE <value>    - Set safe temperature limit");
  logln("  SET P1 <value>      - Set calibration point 1");
  logln("  SET P2 <value>      - Set calibration point 2");
  logln("  SET P3 <value>      - Set calibration point 3");
  logln("  SET P4 <value>      - Set calibration point 4");
  logln("  SET SCALE C|F       - Set temperature scale (Celsius/Fahrenheit)");
  logln("  SET SENSOR 100|1000 - Set test sensor type (PT100/PT1000)");
  logln("  SET WIRES 2|3|4     - Set test sensor wiring mode");
  logln("  --------------------------------------------------");
  logln("  HEAT                - Start heating (PID control)");
  logln("  COOL                - Start cooling (PID control)");
  logln("  STOP                - Stop all processes "
        "(heating/cooling/calibration)");
  logln("  RUN                 - Start calibration cycle");
  logln(
      "  DEFAULT             - Reset all settings to default (requires SAVE)");
  logln("  SAVE                - Save current settings to LittleFS");
  logln("  SHOW                - Show current settings and temperatures");
  logln("  HELP                - Show this help message");
}

void printTemperatures() {
  // Definimos los valores de temperatura
#ifdef SENSOR_SIMULATION
  float masterTemp = 25.0f;
  float testTemp = 25.5f; // Simulamos una temperatura de sensor de prueba
#else
  float masterTemp = sensors.readMasterTemperature();
  float testTemp = sensors.readTestTemperature();
#endif
  float setpoint = settings.getSetTemperature();
  const char *status =
      (controlState == ControlState::RUNNING) ? "RUNNING" : "STOPPED";

  // Usamos el macro F() para ahorrar RAM.
  logln("====================================");
  logln("          CURRENT STATUS");
  logln("====================================");
  log("  SETPOINT:    ");
  logf("%.2f\n", setpoint);
  log("  MASTER TEMP: ");
  logf("%.2f\n", masterTemp);
  log("  TEST TEMP:   ");
  logf("%.2f\n", testTemp);
  logln("------------------------------------");
  logf("  STATUS:      %s\n", status);
  logln("====================================");
}

void printSettings() {
  logln("====================================");
  logln("       CALIBRATOR SETTINGS");
  logln("====================================");

  // --- CONTROL & ALARMAS ---
  logln("--- [CONTROL & ALARMAS] ---");
  logf("  SETPOINT: %.2f\n", settings.getSetTemperature());
  logf("  Alarm High: %.2f\n", settings.getAlarmUpperLimit());
  logf("  Alarm Low: %.2f\n", settings.getAlarmLowerLimit());
  logf("  Danger Temp.: %.2f\n", settings.getDangerTemperature());
  logf("  Safe Temp.: %.2f\n", settings.getSafeTemperature());

  // --- PID PARAMETERS ---
  logln("--- [PID PARAMETERS] ---");
  logf("  KP: %.2f\n", settings.getPidKp());
  logf("  TI: %.2f s\n", settings.getPidTi());
  logf("  TD: %.2f s\n", settings.getPidTd());
  logf("  PID Period: %.2f s\n", settings.getPidPeriod());
  logf("  Stability Time: %.2f s\n", settings.getStabilityTime());

  // --- SENSOR CONFIG ---
  logln("--- [SENSOR CONFIGURATION] ---");
  logf("  Sensor Type: %s\n",
       settings.getSensorType() == SensorType::PT100 ? "PT100" : "PT1000");

  logf("  Sensor Wires: %d\n", settings.getSensorWires());

  logf("  Temp. Scale: %s\n",
       settings.getTemperatureScale() == TemperatureScale::CELSIUS
           ? "Celsius"
           : "Fahrenheit");

  logf("  Master Cal. Offset: %.2f\n", settings.getMasterOffset());
  logf("  Test Cal. Offset: %.2f\n", settings.getTestOffset());
  logf("  Calibration tolerance: %.2f\n", settings.getCalibrationTolerance());

  // --- CALIBRATION POINTS ---
  logln("--- [CALIBRATION POINTS] ---");
  logf("  Calib. P1: %.2f\n", settings.getCalibrationPoint(0));
  logf("  Calib. P2: %.2f\n", settings.getCalibrationPoint(1));
  logf("  Calib. P3: %.2f\n", settings.getCalibrationPoint(2));
  logf("  Calib. P4: %.2f\n", settings.getCalibrationPoint(3));

  logln("====================================");
}

void renderNavBarUI() {
  SensorType sensor = settings.getSensorType();
  if (sensor == SensorType::PT1000) {
    // Si es PT100, establecer pt_0 como activo (true)
    pt_0.setActive(true);
    pt_1.setActive(true);
    pt_2.setActive(true);
    pt_3.setActive(true);
    pt_4.setActive(true);
    pt_5.setActive(true);
  } else if (sensor == SensorType::PT100) {
    // Si es PT1000, establecer pt_0 como inactivo (false)
    pt_0.setActive(false);
    pt_1.setActive(false);
    pt_2.setActive(false);
    pt_3.setActive(false);
    pt_4.setActive(false);
    pt_5.setActive(false);
  }

  TemperatureScale scale = settings.getTemperatureScale();
  if (scale == TemperatureScale::CELSIUS) {
    scale_0.setActive(false);
    scale_1.setActive(false);
    scale_2.setActive(false);
    scale_3.setActive(false);
    scale_4.setActive(false);
    scale_5.setActive(false);
  } else if (scale == TemperatureScale::FAHRENHEIT) {
    scale_0.setActive(true);
    scale_1.setActive(true);
    scale_2.setActive(true);
    scale_3.setActive(true);
    scale_4.setActive(true);
    scale_5.setActive(true);
  }
}

void renderConfigUI(u_int8_t page) {
  char bufferDefault[16];

  if (page == 3) {
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f", settings.getPidKp());
    kp.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f", settings.getPidTi());
    ti.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f", settings.getPidTd());
    td.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%d",
             settings.getPidPeriod());
    period.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%d",
             settings.getStabilityTime());
    stability.setText(bufferDefault);
  } else if (page == 4) {
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f",
             settings.getCalibrationPoint(0));
    setp1.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f",
             settings.getCalibrationPoint(1));
    setp2.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f",
             settings.getCalibrationPoint(2));
    setp3.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f",
             settings.getCalibrationPoint(3));
    setp4.setText(bufferDefault);
  } else if (page == 5) {
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f",
             settings.getMasterOffset());
    moffset.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f",
             settings.getTestOffset());
    toffset.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f",
             settings.getAlarmUpperLimit());
    upperlimit.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f",
             settings.getAlarmLowerLimit());
    lowerlimit.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f",
             settings.getDangerTemperature());
    danger.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f",
             settings.getSafeTemperature());
    safe.setText(bufferDefault);
  }
}
