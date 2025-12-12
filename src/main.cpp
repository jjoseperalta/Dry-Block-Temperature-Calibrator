#include "Buzzer.h"
#include "Calibration.h"
#include "HMIController.h"
#include "Heater.h"
#include "PIDController.h"
#include "Sensors.h"
#include "Settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <Arduino.h>

// Function prototypes
void handleCommand(String command);
void printHelp();
void printSettings();
void printTemperatures();
void renderNavBarUI();
void renderConfigUI(u_int8_t page);

// Prototipos de las tareas de FreeRTOS
void taskControlCore(void *parameter);
void taskInterfaceCore(void *parameter);

// Global objects
Settings settings;
Sensors sensors(settings);
Heater heater;
Buzzer buzzer;
PIDController pid(settings);
Calibration calibration(settings, sensors, buzzer);
HMIController hmi;

enum class ControlState { STOPPED, RUNNING };

ControlState controlState = ControlState::STOPPED;
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

void callbackSensorType(NextionEventType type, INextionTouchable *widget) {
  Serial.println("Callback Sensor Type triggered");
  if (type == NEX_EVENT_POP) {
    NextionDualStateButton *btn = static_cast<NextionDualStateButton *>(widget);

    btn->isActive() ? settings.setSensorType(SensorType::PT1000)
                    : settings.setSensorType(SensorType::PT100);

    Serial.println("Sensor type set to: " +
                   String(settings.getSensorType() == SensorType::PT1000
                              ? "PT1000"
                              : "PT100"));

    settings.save();
  }
}

void callbackTemperatureScale(NextionEventType type,
                              INextionTouchable *widget) {
  Serial.println("Callback Temperature Scale triggered");
  if (type == NEX_EVENT_POP) {
    NextionDualStateButton *btn = static_cast<NextionDualStateButton *>(widget);

    btn->isActive() ? settings.setTemperatureScale(TemperatureScale::FAHRENHEIT)
                    : settings.setTemperatureScale(TemperatureScale::CELSIUS);

    Serial.println(
        "Temperature scale set to: " +
        String(settings.getTemperatureScale() == TemperatureScale::FAHRENHEIT
                   ? "Fahrenheit"
                   : "Celsius"));

    settings.save();
  }
}

void callbackPage0(NextionEventType type, INextionTouchable *widget) {
  Serial.println("Callback Page 0 triggered");
  if (type == NEX_EVENT_PUSH) {
    pt_0.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_0.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    pt_0.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_0.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
  }
}

void callbackPage1(NextionEventType type, INextionTouchable *widget) {
  Serial.println("Callback Page 1 triggered");
  if (type == NEX_EVENT_PUSH) {
    pt_1.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_1.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    pt_1.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_1.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);

    char buffer[16];
    sprintf(buffer, "%.2f", settings.getSetTemperature());
    setp.setText(buffer);
    setp.setText(buffer);

    Serial.println("setp value: " + String(settings.getSetTemperature()));
  }
}

void callbackPage2(NextionEventType type, INextionTouchable *widget) {
  Serial.println("Callback Page 2 triggered");
  if (type == NEX_EVENT_PUSH) {
    pt_2.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_2.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    pt_2.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_2.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
  }
}

void callbackPage3(NextionEventType type, INextionTouchable *widget) {
  Serial.println("Callback Page 3 triggered");
  if (type == NEX_EVENT_PUSH) {
    pt_3.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_3.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    pt_3.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_3.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    renderConfigUI(3);
    renderConfigUI(3);
  }
}

void callbackPage4(NextionEventType type, INextionTouchable *widget) {
  Serial.println("Callback Page 4 triggered");
  if (type == NEX_EVENT_PUSH) {
    pt_4.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_4.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    pt_4.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_4.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    renderConfigUI(4);
    renderConfigUI(4);
  }
}

void callbackPage5(NextionEventType type, INextionTouchable *widget) {
  Serial.println("Callback Page 5 triggered");
  if (type == NEX_EVENT_PUSH) {
    pt_5.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_5.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    pt_5.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_5.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    renderConfigUI(5);
    renderConfigUI(5);
  }
}

void callbackQuickTest(NextionEventType type, INextionTouchable *widget) {
  Serial.println("Callback Quick Test triggered");
  if (type == NEX_EVENT_POP) {
    u_int8_t indexBtn = widget->getComponentID();

    if (indexBtn == heat.getComponentID() ||
        indexBtn == cool.getComponentID()) {

      settings.setSetTemperature(
          setp.getText(buffer, buffer_size) ? atof(buffer) : 0.0f);

      Serial.println("SETP set to: " + String(settings.getSetTemperature(), 2));

      controlState = ControlState::RUNNING;

      pid.reset();

      Serial.println("PID control started. Target: " +
                     String(settings.getSetTemperature(), 2));

    } else if (indexBtn == stop_1.getComponentID()) {
      controlState = ControlState::STOPPED;

      heater.stop();

      calibration.stop();

      Serial.println("All processes stopped.");
    }
  }
}

void callbackAdvancedTest(NextionEventType type, INextionTouchable *widget) {
  Serial.println("Callback Advanced Test triggered");
  if (type == NEX_EVENT_POP) {
    u_int8_t indexBtn = widget->getComponentID();

    if (indexBtn == run.getComponentID()) {

      controlState = ControlState::RUNNING;

      pid.reset();

      calibration.start();

      Serial.println("Calibration started.");

    } else if (indexBtn == stop_2.getComponentID()) {
      controlState = ControlState::STOPPED;

      heater.stop();

      calibration.stop();

      Serial.println("All processes stopped.");
    }
  }
}

void callbackConfig(NextionEventType type, INextionTouchable *widget) {
  Serial.println("Callback Config triggered");
  if (type == NEX_EVENT_POP) {
    u_int8_t indexBtn = widget->getComponentID();

    if (indexBtn == default_3.getComponentID() ||
        indexBtn == default_4.getComponentID() ||
        indexBtn == default_5.getComponentID()) {

      settings.resetToDefaults();

      // renderConfigUI();

      Serial.println("Settings reset to default values.");

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
      setp4.getText(buffer, sizeof(buffer));
      settings.setCalibrationPoint(
          3, (buffer[0] != '\0') ? atof(buffer) : Settings::getDefaultP4());
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

      Serial.println("Settings saved.");
    }
  }
}

void handleCalibrationUpdate(int index) {
  Serial.print("MAIN: Callback recibido. El punto ");
  Serial.print(index);
  Serial.println(" fue registrado. Actualizando HMI.");

  const CalibrationData &pointData = calibration.getCalibrationData(index);

  // 2. Mostrar los valores en la HMI
  Serial.print("SETP: ");
  Serial.println(pointData.setpoint, 2);
  Serial.print("MASTER: ");
  Serial.println(pointData.masterTemp, 2);
  Serial.print("TEST: ");
  Serial.println(pointData.testTemp, 2);
  Serial.print("DIFF: ");
  Serial.println(pointData.difference, 3);

  snprintf(buffer, buffer_size, "%.1f", pointData.setpoint);
  temp_p1.setText(buffer);
  snprintf(buffer, buffer_size, "%.2f", pointData.masterTemp);
  up_temp_master_p1.setText(buffer);
  snprintf(buffer, buffer_size, "%.2f", pointData.testTemp);
  up_temp_test_p1.setText(buffer);
  snprintf(buffer, buffer_size, "%.3f", pointData.difference);
  up_diff_p1.setText(buffer);
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

  Serial.println("Dry Block Temperature Calibrator");
  Serial.println("Type 'HELP' for a list of commands.");

  // DEBUG: Imprimimos la configuración inmediatamente después de cargar
  Serial.println("--- DEBUG: Configuración cargada ---");
  printSettings();
  Serial.println("------------------------------------");

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
}

void loop() {
  //   // Read serial commands
  //   if (Serial.available() > 0) {
  //     String command = Serial.readStringUntil('\n');
  //     command.trim();
  //     command.toUpperCase();
  //     handleCommand(command);
  //   }

  // #if 0
  //     // Old PID timing: uses settings.getPidPeriod() directly as seconds
  // #endif
  //   // PID control loop (uses real elapsed time dt)
  //   unsigned long now = millis();
  //   masterTemp = sensors.getFilteredMasterTemperature(20);
  //   float setpoint = settings.getSetTemperature();
  //   if (controlState == ControlState::RUNNING &&
  //       (now - lastPidTime >= (unsigned long)settings.getPidPeriod() *
  //       1000UL)) {
  // #ifdef SENSOR_SIMULATION
  //     float masterTemp = 25.0f; // Simulate a constant room temperature
  // #else
  //     // float masterTemp = sensors.readMasterTemperature();
  //     // masterTemp = sensors.getFilteredMasterTemperature(10);
  // #endif
  //     // float setpoint = settings.getSetTemperature();
  //     float dt = (now - lastPidTime) / 1000.0f; // seconds
  //     float output = pid.calculate(setpoint, masterTemp, dt);

  //     // Un-comment the line below for debugging PID values
  //     Serial.println("Output: " + String(output) + " | Temp: " +
  //                    String(masterTemp) + " | Setpoint: " +
  //                    String(setpoint));

  //     if (output >= 0) {
  //       // We need to heat or maintain temperature
  //       heater.setCool(0); // Ensure cooling is off
  //       heater.setHeat(output > 100.0f ? 100.0f
  //                                      : output); // Apply heat, capped at 100
  //     } else {
  //       // We need to cool
  //       heater.setCool(fabsf(output) > 100.0f
  //                          ? 100.0f
  //                          : fabsf(output)); // Apply cooling, capped at 100
  //       heater.setHeat(0);                   // Ensure heating is off
  //     }

  //     // // Check if target temperature is reached
  //     // if (abs(masterTemp - setpoint) < 0.5) {
  //     //   if (!targetReachedNotified) {
  //     //     buzzer.beep(BeepType::TARGET_REACHED);
  //     //     targetReachedNotified = true;
  //     //     if (calibration.isRunning()) {
  //     //       calibration.targetReached();
  //     //     }
  //     //   }
  //     // } else {
  //     //   targetReachedNotified = false;
  //     // }

  //     // // Check for alarms
  //     // if (masterTemp > settings.getAlarmUpperLimit() ||
  //     //     masterTemp < settings.getAlarmLowerLimit()) {
  //     //   buzzer.beep(BeepType::ALARM);
  //     // }

  //     lastPidTime = now;
  //   } else if (controlState == ControlState::STOPPED) {
  //     heater.stop(); // Ensure heater is off when stopped
  //   }

  //   // Check if target temperature is reached
  //   if (abs(masterTemp - setpoint) < 0.5) {
  //     if (!targetReachedNotified) {
  //       buzzer.beep(BeepType::TARGET_REACHED);
  //       targetReachedNotified = true;
  //       if (calibration.isRunning()) {
  //         calibration.targetReached();
  //       }
  //     }
  //   } else {
  //     targetReachedNotified = false;
  //   }

  //   // Check for alarms
  //   if (masterTemp > settings.getAlarmUpperLimit() ||
  //       masterTemp < settings.getAlarmLowerLimit()) {
  //     buzzer.beep(BeepType::ALARM);
  //   }

  //   // Calibration loop
  //   calibration.loop();

  //   hmi.poll();

  //   buzzer.handle();

  //   unsigned long tiempoActual = millis();
  //   if (tiempoActual - tiempoAnterior >= intervalo) {
  //     tiempoAnterior = tiempoActual;

  //     const char *unit =
  //         settings.getTemperatureScale() == TemperatureScale::CELSIUS ? "C" :
  //         "F";
  //     // Convertir el contador a cadena
  //     //   sprintf(buffer, "%.2f \xB0%s", sensors.readMasterTemperature(), unit);

  //     // Mostrar en Serial (para depuración)
  //     //   Serial.print("Temperatura: ");
  //     //   Serial.println(sensors.readMasterTemperature());

  //     // snprintf(buffer, buffer_size, "%.2f \xB0%s",
  //     //          sensors.readMasterTemperature(), unit);
  //     snprintf(buffer, buffer_size, "%.1f \xB0%s", masterTemp, unit);
  //     temp_1.setText(buffer);

  //     snprintf(buffer, buffer_size, "%.1f/%.1f", masterTemp,
  //              settings.getSetTemperature());
  //     temp_2.setText(buffer);
  //   }
}

// Tarea 1: Control y Sensores (Core 1)
// Necesitas un Mutex para proteger masterTemp, declarado globalmente:
// SemaphoreHandle_t masterTempMutex;

void taskControlCore(void *parameter) {
  // Variable para manejar la cadencia PID exacta de FreeRTOS
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // El período PID se define en settings (segundos), lo convertimos a Ticks
  TickType_t xPidFrequency = pdMS_TO_TICKS(settings.getPidPeriod() * 1000UL);

  while (true) {
    // --- CÁLCULO PID Y CONTROL (BLOQUE DE CADA PID PERIOD) ---

    // Esperamos con precisión hasta el siguiente ciclo PID.
    vTaskDelayUntil(&xLastWakeTime, xPidFrequency);

    // 1. Obtener Lectura de Sensor (CRÍTICA)
    float currentMasterTemp = sensors.getFilteredMasterTemperature(20);

    // Aseguramos la escritura de masterTemp (CRÍTICO: Core 1 escribe, Core 0
    // lee)
    if (xSemaphoreTake(masterTempMutex, (TickType_t)10) == pdTRUE) {
      masterTemp = currentMasterTemp;
      xSemaphoreGive(masterTempMutex);
    }

    float setpoint = settings.getSetTemperature();
    // unsigned long now = millis(); // Para calcular dt

    // 2. Ejecución del PID (Si está corriendo)
    if (controlState == ControlState::RUNNING) {
      float dt =
          xPidFrequency /
          (float)configTICK_RATE_HZ; // Tiempo exacto del ciclo en segundos
      float output = pid.calculate(setpoint, masterTemp, dt);

      // Un-comment the line below for debugging PID values
      Serial.println("Output: " + String(output) + " | Temp: " +
                     String(masterTemp) + " | Setpoint: " + String(setpoint));

      // 3. Actuación del Heater
      if (output >= 0) {
        heater.setCool(0);
        heater.setHeat(output > 100.0f ? 100.0f : output);
      } else {
        heater.setCool(fabsf(output) > 100.0f ? 100.0f : fabsf(output));
        heater.setHeat(0);
      }

      // 4. Lógica de Target Reached (ALARMA/CALIBRACIÓN)
      if (abs(masterTemp - setpoint) < 0.5) {
        if (!targetReachedNotified) {
          Serial.println("Target reached!");
          buzzer.beep(BeepType::TARGET_REACHED);
          targetReachedNotified = true;
          if (calibration.isRunning()) {
            calibration.targetReached();
          }
        }
      } else {
        targetReachedNotified = false;
      }
    }

    // 5. Lógica de STOP y Alarmas (Fuera del RUNNING para que se verifique
    // siempre)
    if (xSemaphoreTake(controlStateMutex, (TickType_t)10) == pdTRUE) {
      if (controlState == ControlState::STOPPED) {
        heater.stop();
      }
      // No hace falta proteger la lectura de controlState en este core si no se
      // modifica simultáneamente por otra parte de esta misma lógica, pero el
      // cambio a STOPPED debe protegerse por si viene de la tarea de comandos.
      xSemaphoreGive(controlStateMutex);
    }

    if (masterTemp > settings.getAlarmUpperLimit() ||
        masterTemp < settings.getAlarmLowerLimit()) {
      buzzer.beep(BeepType::ALARM);
    }

    // 6. Loop de Calibración
    calibration.loop();

    // NOTA: No es necesario usar el Mutex en masterTemp si solo se actualiza
    // una vez por ciclo PID. El riesgo es bajo, pero si deseas máxima
    // seguridad, usa xSemaphoreTake y xSemaphoreGive alrededor de la
    // escritura/lectura.
  }
}

// Tarea 2: Interfaz HMI y Comunicación Serial (Core 0)
void taskInterfaceCore(void *parameter) {
  // --- Configuración de Frecuencias ---

  // Frecuencia para la actualización LENTA de texto en la UI (500ms)
  const TickType_t xInterfaceUpdateFrequency = pdMS_TO_TICKS(500);
  // Frecuencia RÁPIDA para el polling del HMI y Buzzer (10ms es un buen
  // equilibrio)
  const TickType_t xFastPollDelay = pdMS_TO_TICKS(10);

  // Variable para rastrear el tiempo del último despertar (necesaria para el
  // delay)
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Variable para rastrear el tiempo de la última actualización de texto
  // (independiente del delay)
  TickType_t xLastUpdateTime = xTaskGetTickCount();

  while (true) {
    // --- 1. TAREAS DE ALTA FRECUENCIA (Polling HMI, Buzzer, y Comandos
    // Seriales) --- Estas tareas se ejecutarán cada 10ms

    buzzer.handle();
    hmi.poll(); // ¡CRÍTICO! Necesita ejecutarse rápido para los callbacks
                // Nextion.

    // --- COMUNICACIÓN SERIAL (Comandos) ---
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      command.toUpperCase();
      handleCommand(command);
    }

    // --- 2. ACTUALIZACIÓN DE UI (Lógica de 500ms) ---
    TickType_t xNow = xTaskGetTickCount();

    // Comprobamos si ha pasado el tiempo necesario desde la última
    // actualización de texto
    if ((xNow - xLastUpdateTime) >= xInterfaceUpdateFrequency) {

      // --- LECTURA PROTEGIDA DE TEMPERATURA ---
      float tempToDisplay = 0.0f;
      // Se asume que masterTempMutex es un SemaphoreHandle_t
      if (xSemaphoreTake(masterTempMutex, (TickType_t)10) == pdTRUE) {
        tempToDisplay = masterTemp;
        xSemaphoreGive(masterTempMutex);
      }

      // --- FORMATO Y ESCRITURA DE TEXTO HMI ---

      // Obtener la unidad de temperatura
      const char *unit =
          settings.getTemperatureScale() == TemperatureScale::CELSIUS ? "C"
                                                                      : "F";

      // Actualizar Widget temp_1 (Temperatura actual con unidad)
      snprintf(buffer, buffer_size, "%.1f \xB0%s", tempToDisplay, unit);
      temp_1.setText(buffer);

      // Actualizar Widget temp_2 (Temperatura actual / Setpoint)
      snprintf(buffer, buffer_size, "%.1f/%.1f", tempToDisplay,
               settings.getSetTemperature());
      temp_2.setText(buffer);

      // Restablecer el tiempo de la última actualización
      xLastUpdateTime = xNow;
    }

    // --- 3. CEDER TIEMPO (Garantizar la cadencia rápida de 10ms) ---
    // Usamos vTaskDelay para suspender la tarea por un tiempo corto,
    // asegurando que otras tareas de alta prioridad (como la de control) tengan
    // tiempo de CPU.
    vTaskDelay(xFastPollDelay);
  }
}

void handleCommand(String command) {
  if (command.startsWith("SET")) {
    // SET <parameter> <value>
    // Los comandos SET SCALE, SET SENSOR y SET WIRES son de dos palabras,
    // por lo que usamos el substring para manejar ambos formatos.

    int firstSpace = command.indexOf(' ');
    if (firstSpace == -1) {
      Serial.println("Invalid SET command format. Use SET <PARAM> <VALUE>");
      return;
    }

    String part1 = command.substring(firstSpace + 1);
    int secondSpace = part1.indexOf(' ');

    String parameter, valueStr;

    if (secondSpace == -1) {
      // Manejar comandos de una palabra (e.g., SETP, SAVE) que deberían haber
      // sido capturados antes, pero por seguridad
      Serial.println("Invalid SET command format. Missing value.");
      return;
    }

    parameter = part1.substring(0, secondSpace);
    valueStr = part1.substring(secondSpace + 1);

    // Convertir parámetros a mayúsculas para la comparación
    parameter.toUpperCase();
    valueStr.toUpperCase();

    // ***************************************************
    // COMANDOS DE VALOR (Flotante o Entero)
    // ***************************************************

    if (parameter == "KP") {
      settings.setPidKp(valueStr.toFloat());
      Serial.println("KP set to: " + String(settings.getPidKp(), 2));
    } else if (parameter == "TI") {
      settings.setPidTi(valueStr.toFloat());
      Serial.println("TI set to: " + String(settings.getPidTi(), 2));
    } else if (parameter == "TD") {
      settings.setPidTd(valueStr.toFloat());
      Serial.println("TD set to: " + String(settings.getPidTd(), 2));
    } else if (parameter == "SETP") {
      settings.setSetTemperature(valueStr.toFloat());
      Serial.println("SETP set to: " + String(settings.getSetTemperature(), 2));
    } else if (parameter == "MOFFSET") {
      settings.setMasterOffset(valueStr.toFloat());
      Serial.println("MOFFSET set to: " +
                     String(settings.getMasterOffset(), 2));
    } else if (parameter == "TOFFSET") {
      settings.setTestOffset(valueStr.toFloat());
      Serial.println("TOFFSET set to: " + String(settings.getTestOffset(), 2));
    } else if (parameter == "DANGER") {
      settings.setDangerTemperature(valueStr.toFloat());
      Serial.println("DANGER set to: " +
                     String(settings.getDangerTemperature(), 2));
    } else if (parameter == "SAFE") {
      settings.setSafeTemperature(valueStr.toFloat());
      Serial.println("SAFE set to: " +
                     String(settings.getSafeTemperature(), 2));
    } else if (parameter == "HIGH") {
      settings.setAlarmUpperLimit(valueStr.toFloat());
      Serial.println("Alarm High set to: " +
                     String(settings.getAlarmUpperLimit(), 2));
    } else if (parameter == "LOW") {
      settings.setAlarmLowerLimit(valueStr.toFloat());
      Serial.println("Alarm Low set to: " +
                     String(settings.getAlarmLowerLimit(), 2));
    } else if (parameter == "PERIOD") {
      int intValue = valueStr.toInt();
      if (intValue > 0) {
        settings.setPidPeriod(intValue);
        Serial.println("PID Period set to: " + String(settings.getPidPeriod()) +
                       "s");
      } else {
        Serial.println("ERROR: PID Period must be > 0.");
      }
    } else if (parameter == "STABLE") {
      int intValue = valueStr.toInt();
      if (intValue >= 0) {
        settings.setStabilityTime(intValue);
        Serial.println("Stability Time set to: " +
                       String(settings.getStabilityTime()) + "s");
      } else {
        Serial.println("ERROR: Stability Time must be >= 0.");
      }
    } else if (parameter == "P1") {
      settings.setCalibrationPoint(0, valueStr.toFloat());
      Serial.println("P1 set to: " +
                     String(settings.getCalibrationPoint(0), 2));
    } else if (parameter == "P2") {
      settings.setCalibrationPoint(1, valueStr.toFloat());
      Serial.println("P2 set to: " +
                     String(settings.getCalibrationPoint(1), 2));
    } else if (parameter == "P3") {
      settings.setCalibrationPoint(2, valueStr.toFloat());
      Serial.println("P3 set to: " +
                     String(settings.getCalibrationPoint(2), 2));
    } else if (parameter == "P4") {
      settings.setCalibrationPoint(3, valueStr.toFloat());
      Serial.println("P4 set to: " +
                     String(settings.getCalibrationPoint(3), 2));
    }

    // ***************************************************
    // COMANDOS DE OPCIONES (Cuerdas/Enum)
    // ***************************************************
    else if (parameter == "SCALE") {
      if (valueStr == "C") {
        settings.setTemperatureScale(TemperatureScale::CELSIUS);
        Serial.println(F("Scale updated to: Celsius"));
      } else if (valueStr == "F") {
        settings.setTemperatureScale(TemperatureScale::FAHRENHEIT);
        Serial.println(F("Scale updated to: Fahrenheit"));
      } else {
        Serial.println(F("ERROR: Use SET SCALE C or SET SCALE F."));
      }
    } else if (parameter == "SENSOR") {
      SensorType newType = settings.getSensorType();

      if (valueStr == "100") {
        newType = SensorType::PT100;
      } else if (valueStr == "1000") {
        newType = SensorType::PT1000;
      } else {
        Serial.println(F("ERROR: Use SET SENSOR 100 or SET SENSOR 1000."));
        return;
      }

      // Notifica a la clase Sensors para configurar el hardware (manteniendo
      // los hilos)
      sensors.configureTestSensor(newType, settings.getSensorWires());
      Serial.println("Sensor Type updated to: " + valueStr);
    } else if (parameter == "WIRES") {
      int intValue = valueStr.toInt();
      if (intValue >= 2 && intValue <= 4) {
        // Notifica a la clase Sensors para configurar el hardware
        sensors.configureTestSensor(settings.getSensorType(), intValue);
        Serial.println("Sensor Wires updated to: " + String(intValue));
      } else {
        Serial.println(F("ERROR: El numero de hilos debe ser 2, 3 o 4."));
      }
    } else {
      Serial.println("Unknown parameter: " + parameter);
    }

  } else if (command == "HEAT" || command == "COOL") {
    controlState = ControlState::RUNNING;
    pid.reset();
    Serial.println("PID control started. Target: " +
                   String(settings.getSetTemperature(), 2));
  } else if (command == "STOP") {
    controlState = ControlState::STOPPED;
    heater.stop();
    calibration.stop();
    Serial.println("All processes stopped.");
  } else if (command == "RUN") {
    controlState = ControlState::RUNNING;
    pid.reset();
    calibration.start();
    Serial.println("Run processes");
  } else if (command == "DEFAULT") {
    settings.resetToDefaults();
    Serial.println("Restored default settings. REMEMBER TO SAVE!");
  } else if (command == "SAVE") {
    settings.save();
    Serial.println("Saved current settings.");
  } else if (command == "HELP") {
    printHelp();
  } else if (command == "SHOW") {
    printSettings();
  } else if (command == "TEMP") { // ¡NUEVO! Manejar comando TEMP
    printTemperatures();
  } else {
    Serial.println("Unknown command");
  }
}

void printHelp() {
  Serial.println("Available commands:");
  Serial.println("  TEMP                - Show current block temperatures and "
                 "setpoint"); // ¡NUEVO!
  Serial.println("  SET SETP <value>    - Set target temperature");
  Serial.println("  SET KP <value>      - Set Kp for PID");
  Serial.println("  SET TI <value>      - Set Ti for PID (seconds)");
  Serial.println("  SET TD <value>      - Set Td for PID (seconds)");
  Serial.println(
      "  SET PERIOD <value>  - Set PID cycle period (seconds, integer) ");
  Serial.println(
      "  SET STABLE <value>  - Set stability time (seconds, integer)");
  Serial.println("  SET HIGH <value>    - Set high alarm limit");
  Serial.println("  SET LOW <value>     - Set low alarm limit");
  Serial.println(
      "  SET MOFFSET <value> - Set master sensor calibration offset");
  Serial.println(
      "  SET TOFFSET <value> - Set under test sensor calibration offset");
  Serial.println("  SET DANGER <value>  - Set danger temperature limit");
  Serial.println("  SET SAFE <value>    - Set safe temperature limit");
  Serial.println("  SET P1 <value>      - Set calibration point 1");
  Serial.println("  SET P2 <value>      - Set calibration point 2");
  Serial.println("  SET P3 <value>      - Set calibration point 3");
  Serial.println("  SET P4 <value>      - Set calibration point 4");
  Serial.println(
      "  SET SCALE C|F       - Set temperature scale (Celsius/Fahrenheit)");
  Serial.println("  SET SENSOR 100|1000 - Set test sensor type (PT100/PT1000)");
  Serial.println("  SET WIRES 2|3|4     - Set test sensor wiring mode");
  Serial.println("  --------------------------------------------------");
  Serial.println("  HEAT                - Start heating (PID control)");
  Serial.println("  COOL                - Start cooling (PID control)");
  Serial.println("  STOP                - Stop all processes "
                 "(heating/cooling/calibration)");
  Serial.println("  RUN                 - Start calibration cycle");
  Serial.println(
      "  DEFAULT             - Reset all settings to default (requires SAVE)");
  Serial.println("  SAVE                - Save current settings to LittleFS");
  Serial.println(
      "  SHOW                - Show current settings and temperatures");
  Serial.println("  HELP                - Show this help message");
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
  String status =
      (controlState == ControlState::RUNNING) ? "RUNNING" : "STOPPED";

  // Usamos el macro F() para ahorrar RAM.
  Serial.println(F("===================================="));
  Serial.println(F("          CURRENT STATUS"));
  Serial.println(F("===================================="));
  Serial.print(F("  SETPOINT:    "));
  Serial.println(String(setpoint, 2));
  Serial.print(F("  MASTER TEMP: "));
  Serial.println(String(masterTemp, 2));
  Serial.print(F("  TEST TEMP:   "));
  Serial.println(String(testTemp, 2));
  Serial.println(F("------------------------------------"));
  Serial.println("  STATUS:      " + status);
  Serial.println(F("===================================="));
}

void printSettings() {
  // Usamos el macro F() para ahorrar RAM y el constructor String(float,
  // decimales) para forzar la precisión.
  Serial.println(F("===================================="));
  Serial.println(F("       CALIBRATOR SETTINGS"));
  Serial.println(F("===================================="));

  // --- CONTROL & ALARMAS ---
  Serial.println(F("--- [CONTROL & ALARMAS] ---"));
  Serial.print(F("  SETPOINT: "));
  Serial.println(String(settings.getSetTemperature(), 2));
  Serial.print(F("  Alarm High: "));
  Serial.println(String(settings.getAlarmUpperLimit(), 2));
  Serial.print(F("  Alarm Low: "));
  Serial.println(String(settings.getAlarmLowerLimit(), 2));
  Serial.print(F("  Danger Temp.: "));
  Serial.println(String(settings.getDangerTemperature(), 2));
  Serial.print(F("  Safe Temp.: "));
  Serial.println(String(settings.getSafeTemperature(), 2));

  // --- PID PARAMETERS ---
  Serial.println(F("--- [PID PARAMETERS] ---"));
  Serial.print(F("  KP: "));
  Serial.println(String(settings.getPidKp(), 2));
  Serial.print(F("  TI: "));
  Serial.print(String(settings.getPidTi(), 2));
  Serial.println(F("s"));
  Serial.print(F("  TD: "));
  Serial.print(String(settings.getPidTd(), 2));
  Serial.println(F("s"));
  Serial.print(F("  PID Period: "));
  Serial.print(settings.getPidPeriod());
  Serial.println(F("s"));
  Serial.print(F("  Stability Time: "));
  Serial.print(settings.getStabilityTime());
  Serial.println(F("s"));

  // --- SENSOR CONFIG ---
  Serial.println(F("--- [SENSOR CONFIGURATION] ---"));
  String sensorTypeStr =
      (settings.getSensorType() == SensorType::PT100) ? "PT100" : "PT1000";
  Serial.println("  Sensor Type: " + sensorTypeStr);
  Serial.println("  Sensor Wires: " + String(settings.getSensorWires()));
  String tempScaleStr =
      (settings.getTemperatureScale() == TemperatureScale::CELSIUS)
          ? "Celsius"
          : "Fahrenheit";
  Serial.println("  Temp. Scale: " + tempScaleStr);
  Serial.print(F("  Master Cal. Offset: "));
  Serial.println(String(settings.getMasterOffset(), 2));
  Serial.print(F("  Test Cal. Offset: "));
  Serial.println(String(settings.getTestOffset(), 2));

  // --- CALIBRATION POINTS ---
  Serial.println(F("--- [CALIBRATION POINTS] ---"));
  Serial.print(F("  Calib. P1: "));
  Serial.println(String(settings.getCalibrationPoint(0), 2));
  Serial.print(F("  Calib. P2: "));
  Serial.println(String(settings.getCalibrationPoint(1), 2));
  Serial.print(F("  Calib. P3: "));
  Serial.println(String(settings.getCalibrationPoint(2), 2));
  Serial.print(F("  Calib. P4: "));
  Serial.println(String(settings.getCalibrationPoint(3), 2));

  Serial.println(F("===================================="));
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