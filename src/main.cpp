#include "AmbientSensor.h"
#include "Buzzer.h"
#include "Calibration.h"
#include "Fan.h"
#include "HMIController.h"
#include "Heater.h"
#include "Logger.h"
#include "PIDAutotuner.h"
#include "PIDController.h"
#include "Sensors.h"
#include "Settings.h"
#include "TimeManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <cmath>

#define CMD_BUFFER_SIZE 64

// Function prototypes
void updatePage0UI();
void callbackPage0(NextionEventType type, INextionTouchable *widget);
void handleCommand(const char *command);
void printHelp();
void printSettings();
void printTemperatures();
void renderNavBarUI();
void renderConfigUI(u_int8_t page);
bool isStableEnoughForCalibration(float masterTemp, float setpoint);
void startThermalProcess();
void stopThermalProcess();
void checkAndHandleSetpointStability(float masterTemp, float setpoint);
void renderStopButtonUI();
void parseTempAndUnit(char *input, float &temp, char &unit);
// Prototipos de las tareas de FreeRTOS
void taskControlCore(void *parameter);
void taskInterfaceCore(void *parameter);
void taskConsole(void *parameter);

// Global objects
AmbientSensor ambientSensor;
Settings settings;
Sensors sensors(settings);
Fan fan;
Heater heater(fan);
Buzzer buzzer;
PIDAutotuner autotuner(heater, sensors);
PIDController pid(settings);
Calibration calibration(settings, buzzer);
HMIController hmi;
TimeManager timeManager;

// enum class ThermalState { RAMPING, APPROACHING, HOLDING };
// static ThermalState thermalState = ThermalState::RAMPING;
// const char *stateNames[] = {"RAMPING", "APPROACHING", "HOLDING"};
enum class ControlState {
  STOPPED,
  RUNNING,
  IDLE,
  TIMEOUT,
  CALIBRATING,
  AUTOTUNING
};
static ControlState controlState = ControlState::IDLE;

// Variables para el Hold Timer
static unsigned long holdStartTime = 0;
static bool holdTimerActive = false;

const long intervalo = 500;
unsigned long tiempoAnterior = 0;

char buffer[32];
size_t buffer_size = sizeof(buffer);

float masterTemp = 0.0f;
float testTemp = 0.0f;

unsigned long processStartTime;
bool processRunning = false;

// VARIABLES Y OBJETOS DE FREERTOS
SemaphoreHandle_t controlStateMutex; // Mutex para proteger controlState
SemaphoreHandle_t masterTempMutex;   // Mutex para proteger masterTemp
SemaphoreHandle_t testTempMutex;
SemaphoreHandle_t serialMutex; // Mutex para proteger acceso a Serial

void callbackSensorType(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Sensor Type triggered");
  if (type == NEX_EVENT_POP) {
    NextionDualStateButton *btn = static_cast<NextionDualStateButton *>(widget);

    btn->isActive() ? settings.setSensorType(SensorType::PT1000)
                    : settings.setSensorType(SensorType::PT100);

    logf("Sensor type set to: %s\n",
         settings.getSensorType() == SensorType::PT1000 ? "PT1000" : "PT100");

    settings.save();
    buzzer.playBlocking(BeepType::ACK);
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
    buzzer.playBlocking(BeepType::ACK);
  }
}

void updatePage0UI() {
  pt_0.setActive(settings.getSensorType() == SensorType::PT1000);
  pt_0.setActive(settings.getSensorType() == SensorType::PT1000);
  scale_0.setActive(settings.getTemperatureScale() ==
                    TemperatureScale::FAHRENHEIT);
  scale_0.setActive(settings.getTemperatureScale() ==
                    TemperatureScale::FAHRENHEIT);

  timeManager.updateHMI();

  float setpointDisplay = settings.getSetpoint();
  if (settings.getTemperatureScale() == TemperatureScale::FAHRENHEIT) {
    setpointDisplay = setpointDisplay * 1.8f + 32.0f;
  }
  snprintf(buffer, sizeof(buffer), "%.2f", setpointDisplay);
  setpoint_0.setText(buffer);
  setpoint_0.setText(buffer);

  logf("SETPOINT displayed as: %s %s\n", buffer,
       settings.getTemperatureScale() == TemperatureScale::CELSIUS ? "°C"
                                                                   : "°F");
}

void callbackPage0(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Page 0 triggered");
  if (type == NEX_EVENT_PUSH || type == NEX_EVENT_POP) {
    updatePage0UI();
  }
}

void callbackPage1(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Page 1 triggered");
  if (type == NEX_EVENT_PUSH || type == NEX_EVENT_POP) {
    pt_1.setActive(settings.getSensorType() == SensorType::PT1000);
    pt_1.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_1.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    scale_1.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);

    timeManager.updateHMI();
  }
}

void callbackPage2(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Page 2 triggered");
  if (type == NEX_EVENT_PUSH || type == NEX_EVENT_POP) {
    pt_2.setActive(settings.getSensorType() == SensorType::PT1000);
    pt_2.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_2.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    scale_2.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);

    timeManager.updateHMI();

    renderConfigUI(2);
    // renderConfigUI(2);
  }
}

void callbackPage3(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Page 3 triggered");
  if (type == NEX_EVENT_PUSH || type == NEX_EVENT_POP) {
    pt_3.setActive(settings.getSensorType() == SensorType::PT1000);
    pt_3.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_3.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    scale_3.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);

    timeManager.updateHMI();

    renderConfigUI(3);
    // renderConfigUI(3);
  }
}

void callbackPage4(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Page 4 triggered");
  if (type == NEX_EVENT_PUSH || type == NEX_EVENT_POP) {
    pt_4.setActive(settings.getSensorType() == SensorType::PT1000);
    pt_4.setActive(settings.getSensorType() == SensorType::PT1000);
    scale_4.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);
    scale_4.setActive(settings.getTemperatureScale() ==
                      TemperatureScale::FAHRENHEIT);

    timeManager.updateHMI();

    renderConfigUI(4);
    // renderConfigUI(4);
  }
}

void callbackQuickTest(NextionEventType type, INextionTouchable *widget) {
  logln("Callback Quick Test triggered");
  if (type == NEX_EVENT_POP || type == NEX_EVENT_PUSH) {
    u_int8_t indexBtn = widget->getComponentID();
    // logf("Button ID: %d\n", indexBtn);

    if (indexBtn == set.getComponentID()) { // Botón START
      // logln("Starting thermal process...");

      float sp = setpoint_0.getText(buffer, buffer_size)
                     ? atof(buffer)
                     : Settings::getDefaultSetpoint();
      if (settings.getTemperatureScale() == TemperatureScale::FAHRENHEIT) {
        sp = (sp - 32) * 5 / 9;
      }

      // logf("Parsed setpoint: %.2f (from input: %s)\n", sp, buffer);

      settings.setSetpoint(sp);

      char displayUnit =
          (settings.getTemperatureScale() == TemperatureScale::CELSIUS) ? 'C'
                                                                        : 'F';

      logf("SETPOINT set to: %.2f %c\n", settings.getSetpoint(displayUnit),
           displayUnit);

      startThermalProcess();
      buzzer.playBlocking(BeepType::START_PROCESS);

    } else if (indexBtn == stop_0.getComponentID()) { // Botón STOP
      stopThermalProcess();
      buzzer.playBlocking(BeepType::STOP);
      logln("Process stopped by user.");
    }
  }
}

void callbackAdvancedTest(NextionEventType type, INextionTouchable *widget) {
  // logln("Callback Advanced Test triggered");
  if (type == NEX_EVENT_POP) {
    u_int8_t indexBtn = widget->getComponentID();
    // logf("Button ID: %d\n", indexBtn);
    if (indexBtn == run_1.getComponentID()) {
      controlState = ControlState::RUNNING;
      pid.reset();
      pid.setPreviousPV(masterTemp);
      calibration.start();
      logln("Calibration started.");
      buzzer.playBlocking(BeepType::START_PROCESS);
    } else if (indexBtn == stop_1.getComponentID()) {
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

    if (indexBtn == default_2.getComponentID() ||
        indexBtn == default_3.getComponentID() ||
        indexBtn == default_4.getComponentID()) {

      settings.resetToDefaults();
      settings.save();
      buzzer.playBlocking(BeepType::ACK);
      logln("Settings reset to default values.");

    } else if (indexBtn == save_2.getComponentID()) {

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
      stable.getText(buffer, sizeof(buffer));
      settings.setStabilityTime((buffer[0] != '\0')
                                    ? atoi(buffer)
                                    : Settings::getDefaultStabilityTime());
      settings.save();
      buzzer.playBlocking(BeepType::ACK);
      logln("Settings PID saved.");

    } else if (indexBtn == save_3.getComponentID()) {
      char buffer[16];
      setp1.getText(buffer, sizeof(buffer));
      settings.setCalibrationPoint(
          0, (buffer[0] != '\0') ? atof(buffer) : Settings::getDefaultP1());
      setp2.getText(buffer, sizeof(buffer));
      settings.setCalibrationPoint(
          1, (buffer[0] != '\0') ? atof(buffer) : Settings::getDefaultP2());
      setp3.getText(buffer, sizeof(buffer));
      settings.setCalibrationPoint(
          2, (buffer[0] != '\0') ? atof(buffer) : Settings::getDefaultP3());
      settings.save();
      buzzer.playBlocking(BeepType::ACK);
      logln("Settings SETPOINT's saved.");

    } else if (indexBtn == save_4.getComponentID()) {
      char buffer[16];
      moffset.getText(buffer, sizeof(buffer));
      settings.setMasterCorrections(
          1, (buffer[0] != '\0') ? atof(buffer)
                                 : Settings::getDefaultMasterCorrections(1));
      toffset.getText(buffer, sizeof(buffer));
      settings.setTestCorrections(
          1, (buffer[0] != '\0') ? atof(buffer)
                                 : Settings::getDefaultTestCorrections(1));
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
      buzzer.playBlocking(BeepType::ACK);
      logln("Settings OFFSET/ALARM/LIMIT saved.");
    }
  }
}

void handleCalibrationUpdate(int index) {
  logf("MAIN: Callback recibido. El punto %i fue registrado.", index);
  pid.reset(); // Reiniciar PID para que actúe rápido ante los cambios de
               // setpoint

  const CalibrationData &pointData = calibration.getCalibrationData(index);

  // Determinar la unidad actual del sistema
  bool isFahrenheit =
      (settings.getTemperatureScale() == TemperatureScale::FAHRENHEIT);
  const char *unit = isFahrenheit ? "F" : "C";

  // Convertir valores a la unidad actual para mostrar
  float setpointDisplay = pointData.setpoint;
  float masterDisplay = pointData.masterTemp;
  float testDisplay = pointData.testTemp;
  float diffDisplay = pointData.difference; // La diferencia también se escala

  if (isFahrenheit) {
    setpointDisplay = pointData.setpoint * 1.8f + 32.0f;
    masterDisplay = pointData.masterTemp * 1.8f + 32.0f;
    testDisplay = pointData.testTemp * 1.8f + 32.0f;
    diffDisplay = pointData.difference * 1.8f; // Diferencia solo multiplicar
  }

  // Log en ambas unidades para depuración
  if (isFahrenheit) {
    logf("SETP: %.1f°F (%.1f°C) | MASTER: %.2f°F (%.2f°C) | TEST: %.2f°F "
         "(%.2f°C) | DIFF: %.3f°F (%.3f°C)\n",
         setpointDisplay, pointData.setpoint, masterDisplay,
         pointData.masterTemp, testDisplay, pointData.testTemp, diffDisplay,
         pointData.difference);
  } else {
    logf("SETP: %.1f°C | MASTER: %.2f°C | TEST: %.2f°C | DIFF: %.3f°C\n",
         pointData.setpoint, pointData.masterTemp, pointData.testTemp,
         pointData.difference);
  }

  // Mostrar valores en la HMI (convertidos a la unidad actual)
  char unitBuffer[8];
  snprintf(unitBuffer, sizeof(unitBuffer), "°%s", unit);

  if (index == 0) {
    snprintf(buffer, buffer_size, "%.1f", setpointDisplay);
    temp_p1.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", masterDisplay);
    master_p1.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", testDisplay);
    test_p1.setText(buffer);
    snprintf(buffer, buffer_size, "%+.3f", diffDisplay);
    diff_p1.setText(buffer);
    snprintf(buffer, buffer_size, "%s", pointData.isRising ? "UP" : "DOWN");
    dir_p1.setText(buffer);
  } else if (index == 1) {
    snprintf(buffer, buffer_size, "%.1f", setpointDisplay);
    temp_p2.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", masterDisplay);
    master_p2.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", testDisplay);
    test_p2.setText(buffer);
    snprintf(buffer, buffer_size, "%+.3f", diffDisplay);
    diff_p2.setText(buffer);
    snprintf(buffer, buffer_size, "%s", pointData.isRising ? "UP" : "DOWN");
    dir_p2.setText(buffer);
  } else if (index == 2) {
    snprintf(buffer, buffer_size, "%.1f", setpointDisplay);
    temp_p3.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", masterDisplay);
    master_p3.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", testDisplay);
    test_p3.setText(buffer);
    snprintf(buffer, buffer_size, "%+.3f", diffDisplay);
    diff_p3.setText(buffer);
    snprintf(buffer, buffer_size, "%s", pointData.isRising ? "UP" : "DOWN");
    dir_p3.setText(buffer);
  } else if (index == 3) {
    snprintf(buffer, buffer_size, "%.1f", setpointDisplay);
    temp_p4.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", masterDisplay);
    master_p4.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", testDisplay);
    test_p4.setText(buffer);
    snprintf(buffer, buffer_size, "%+.3f", diffDisplay);
    diff_p4.setText(buffer);
    snprintf(buffer, buffer_size, "%s", pointData.isRising ? "UP" : "DOWN");
    dir_p4.setText(buffer);
  } else if (index == 4) {
    snprintf(buffer, buffer_size, "%.1f", setpointDisplay);
    temp_p5.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", masterDisplay);
    master_p5.setText(buffer);
    snprintf(buffer, buffer_size, "%.2f", testDisplay);
    test_p5.setText(buffer);
    snprintf(buffer, buffer_size, "%+.3f", diffDisplay);
    diff_p5.setText(buffer);
    snprintf(buffer, buffer_size, "%s", pointData.isRising ? "UP" : "DOWN");
    dir_p5.setText(buffer);
  }
}

void setup() {
  Serial.setRxBufferSize(2048); // Aumentar buffer RX
  Serial.setTxBufferSize(2048); // Aumentar buffer TX

  Serial.begin(115200);

  while (!Serial) {
    delay(10); // wait for serial port to connect.
  }

  // Configuración importante para ESP32
  Serial.setTimeout(10); // Timeout para operaciones de lectura

  // Llama a Settings::begin(), que monta LittleFS y llama a settings.load()
  ambientSensor.begin();
  settings.begin();
  sensors.begin();
  heater.begin();
  buzzer.begin();
  hmi.init();
  fan.begin();
  timeManager.begin();

  ambientSensor.setOffset(-3.13f);
  buzzer.playBlocking(BeepType::READY);
  calibration.setRegisterCallback(handleCalibrationUpdate);

  logln("Dry Block Temperature Calibrator");
  logln("Type 'HELP' for a list of commands.");

  // DEBUG: Imprimimos la configuración inmediatamente después de cargar
  logln("--- DEBUG: Configuración cargada ---");
  printSettings();

  // Register callback
  HMIController::registerCallback(&pt_0, callbackSensorType);
  HMIController::registerCallback(&scale_0, callbackTemperatureScale);
  HMIController::registerCallback(&set, callbackQuickTest);
  HMIController::registerCallback(&run_0, callbackPage1);
  HMIController::registerCallback(&stop_0, callbackQuickTest);
  HMIController::registerCallback(&config_0, callbackPage2);

  HMIController::registerCallback(&pt_1, callbackSensorType);
  HMIController::registerCallback(&scale_1, callbackTemperatureScale);
  HMIController::registerCallback(&run_1, callbackAdvancedTest);
  HMIController::registerCallback(&config_1, callbackPage2);
  HMIController::registerCallback(&stop_1, callbackAdvancedTest);
  HMIController::registerCallback(&backhome_1, callbackPage0);

  HMIController::registerCallback(&pt_2, callbackSensorType);
  HMIController::registerCallback(&scale_2, callbackTemperatureScale);
  HMIController::registerCallback(&next_2, callbackPage3);
  HMIController::registerCallback(&default_2, callbackConfig);
  HMIController::registerCallback(&save_2, callbackConfig);
  HMIController::registerCallback(&backhome_2, callbackPage0);

  HMIController::registerCallback(&pt_3, callbackSensorType);
  HMIController::registerCallback(&scale_3, callbackTemperatureScale);
  HMIController::registerCallback(&back_3, callbackPage2);
  HMIController::registerCallback(&next_3, callbackPage4);
  HMIController::registerCallback(&default_3, callbackConfig);
  HMIController::registerCallback(&save_3, callbackConfig);
  HMIController::registerCallback(&backhome_3, callbackPage0);

  HMIController::registerCallback(&pt_4, callbackSensorType);
  HMIController::registerCallback(&scale_4, callbackTemperatureScale);
  HMIController::registerCallback(&back_4, callbackPage3);
  HMIController::registerCallback(&default_4, callbackConfig);
  HMIController::registerCallback(&save_4, callbackConfig);
  HMIController::registerCallback(&backhome_4, callbackPage0);

  controlStateMutex = xSemaphoreCreateMutex();
  masterTempMutex = xSemaphoreCreateMutex();
  testTempMutex = xSemaphoreCreateMutex();
  // serialMutex = xSemaphoreCreateMutex();

  // Crear mutex con prioridad de herencia para evitar inversión
  serialMutex = xSemaphoreCreateMutex();
  if (serialMutex == NULL) {
    // Fallback: sin mutex
    Serial.println("ERROR: No se pudo crear mutex para Serial");
  }

  // Pequeña pausa para estabilizar serial
  delay(100);

  // Limpiar buffers
  while (Serial.available())
    Serial.read();
  Serial.flush();

  // Tarea 1: Control (Core 1 / App Core) - Mayor prioridad para el control
  xTaskCreatePinnedToCore(taskControlCore,          // Función a ejecutar
                          "ControlTask",            // Nombre de la tarea
                          10240,                    // Tamaño de pila (bytes)
                          NULL,                     // Parámetro de la tarea
                          configMAX_PRIORITIES - 1, // Prioridad (Alta)
                          NULL, // Handle de la tarea (No necesario aquí)
                          1     // NÚCLEO 1 (App Core)
  );

  // Tarea 2: Interfaz (Core 0 / Pro Core) - Baja prioridad, permite al SO
  // funcionar
  xTaskCreatePinnedToCore(taskInterfaceCore, // Función a ejecutar
                          "InterfaceTask",   // Nombre de la tarea
                          4096,              // Tamaño de pila (bytes)
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

  updatePage0UI();
}

void loop() {}

// Tarea 1: Control y Sensores (Core 1)
void taskControlCore(void *parameter) {
  static ControlState lastState = ControlState::STOPPED;
  static uint32_t lastLogTime = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    // Cálculo de Frecuencia (Sincronizado con settings)
    TickType_t xPidFrequency = pdMS_TO_TICKS(settings.getPidPeriod() * 1000UL);
    if (xPidFrequency == 0)
      xPidFrequency = pdMS_TO_TICKS(100UL);

    // Obtener Lectura de Sensor
    // float currentMasterTemp = sensors.getFilteredMasterTemperature(0.1f,
    // false); float currentTestTemp = sensors.getFilteredTestTemperature(0.1f,
    // false);
    float currentMasterTemp = sensors.getMasterTemperature();
    float currentTestTemp = sensors.getTestTemperature();

    // Comunicación Inter-Core: Escritura de masterTemp (Protegida)
    if (xSemaphoreTake(masterTempMutex, (TickType_t)10) == pdTRUE) {
      masterTemp = currentMasterTemp;
      xSemaphoreGive(masterTempMutex);
    }

    if (xSemaphoreTake(testTempMutex, (TickType_t)10) == pdTRUE) {
      testTemp = currentTestTemp;
      xSemaphoreGive(testTempMutex);
    }

    // Obtener Estado de Control (Mutex)
    ControlState currentState;
    if (xSemaphoreTake(controlStateMutex, (TickType_t)10) == pdTRUE) {
      currentState = controlState;
      xSemaphoreGive(controlStateMutex);
    } else {
      currentState = ControlState::STOPPED;
    }

    // Manejo de transiciones de estado
    if (currentState != lastState) {
      if (currentState == ControlState::STOPPED) {
        heater.stop();
        fan.stop();
        pid.reset();
        logln("System STOPPED");
      }
      lastState = currentState;
    }

    // Ejecución del PID cuando está en RUNNING
    // Ejecución del PID cuando está en RUNNING
    if (currentState == ControlState::RUNNING) {
      float setpoint = settings.getSetpoint();
      float dt = xPidFrequency / (float)configTICK_RATE_HZ;
      if (dt <= 0)
        dt = 1.0f;

      // Calcular PID
      float output = pid.compute(setpoint, currentMasterTemp, dt,
                                 calibration.isBrakingStep());
      heater.setPower(output);

      // =============================================
      // LÓGICA DE FINALIZACIÓN DE PROCESO
      // =============================================
      if (calibration.isRunning()) {
        // Modo CALIBRACIÓN: tiene su propio timeout interno (MAX_WAIT_TIME_MS)
        calibration.loop(currentMasterTemp, currentTestTemp);
        if (!calibration.isRunning()) {
          stopThermalProcess();
          renderStopButtonUI();
          buzzer.playBlocking(BeepType::SUCCESS);
        }
      } else {
        // Modo NORMAL: verificar estabilidad del setpoint actual
        checkAndHandleSetpointStability(currentMasterTemp, setpoint);

        if (processRunning && (millis() - processStartTime >
                               settings.getMaxProcessTime() * 1000.0f)) {
          stopThermalProcess();
          renderStopButtonUI();
          buzzer.playBlocking(BeepType::ALARM);
          logln("[TIMEOUT] Process stopped - Setpoint not reached within time "
                "limit.");
        }
      }
      // =============================================

      // LOGS (cada 1 segundo)
      uint32_t now = millis();
      if (now - lastLogTime >= 1000) {
        lastLogTime = now;
        float error = setpoint - currentMasterTemp;
        const char *stateStr = pid.getStateString();

        if (calibration.isRunning()) {
          float tempDifference = currentTestTemp - currentMasterTemp;
          logf("State: %s | Error: %.2f | Output: %.2f | Temp: %.2f | Test: "
               "%.2f | Diff: %.2f | Setpoint: %.2f | Stable: %lus/%lus\n",
               stateStr, error, output, currentMasterTemp, currentTestTemp,
               tempDifference, setpoint, calibration.getStableElapsed() / 1000,
               (uint32_t)settings.getStabilityTime());
        } else {
          // Mostrar tiempo restante hasta timeout
          uint32_t elapsed = (millis() - processStartTime) / 1000;
          uint32_t maxTime = settings.getMaxProcessTime();
          uint32_t timeLeft = (elapsed < maxTime) ? (maxTime - elapsed) : 0;
          float tempDifference = currentTestTemp - currentMasterTemp;
          if (holdTimerActive) {
            uint32_t holdElapsed = (millis() - holdStartTime) / 1000;
            logf("State: %s | Error: %.2f | Output: %.2f | Temp: %.2f | Test: %.2f | Diff: %.2f | "
                 "Setpoint: %.2f | Stable: %lu/%lus | Timeout: %lus\n",
                 stateStr, error, output, currentMasterTemp, currentTestTemp,
                 tempDifference, setpoint, calibration.getStableElapsed() / 1000,
                 (uint32_t)settings.getStabilityTime(), timeLeft);
          } else {
            logf("State: %s | Error: %.2f | Output: %.2f | Temp: %.2f | Test: %.2f | Diff: %.2f | "
                 "Setpoint: %.2f\n",
                 stateStr, error, output, currentMasterTemp, currentTestTemp,
                 tempDifference, setpoint);
          }
        }
      }
    }

    // Alarmas Críticas (Siempre activas)
    if (currentMasterTemp > settings.getAlarmUpperLimit() ||
        currentMasterTemp < settings.getAlarmLowerLimit()) {
      buzzer.playBlocking(BeepType::ALARM);
    }

    // Sincronización de tarea
    vTaskDelayUntil(&xLastWakeTime, xPidFrequency);
  }
}

// Tarea 2: Interfaz HMI y Comunicación Serial (Core 0)
void taskInterfaceCore(void *parameter) {
  const TickType_t xFastPollFrequency = pdMS_TO_TICKS(10);
  const int UPDATE_CYCLES = 10; // Actualizar cada 100ms

  TickType_t xLastWakeTime = xTaskGetTickCount();
  int updateCycleCounter = 0;

  // Variables para detectar cambios en el setpoint
  float lastSetpointDisplay = -999.0f;

  while (true) {
    buzzer.handle();
    timeManager.update();
    hmi.poll();

    updateCycleCounter++;
    if (updateCycleCounter >= UPDATE_CYCLES) {
      updateCycleCounter = 0;

      // ============================================================
      // TEMPERATURAS
      // ============================================================
      float tempToDisplay = 0.0f;
      float testTempToDisplay = 0.0f;

      if (xSemaphoreTake(masterTempMutex, (TickType_t)10) == pdTRUE) {
        tempToDisplay = masterTemp;
        xSemaphoreGive(masterTempMutex);
      }

      if (xSemaphoreTake(testTempMutex, (TickType_t)10) == pdTRUE) {
        testTempToDisplay = testTemp;
        xSemaphoreGive(testTempMutex);
      }

      bool isFahrenheit =
          (settings.getTemperatureScale() == TemperatureScale::FAHRENHEIT);
      const char *unit = isFahrenheit ? "F" : "C";

      if (isFahrenheit) {
        tempToDisplay = tempToDisplay * 1.8f + 32.0f;
        testTempToDisplay = testTempToDisplay * 1.8f + 32.0f;
      }

      // Actualizar widgets de temperatura
      snprintf(buffer, buffer_size, "%.2f \xB0%s", tempToDisplay, unit);
      temp_0.setText(buffer);

      snprintf(buffer, buffer_size, "%.2f \xB0%s", testTempToDisplay, unit);
      test_0.setText(buffer);

      snprintf(buffer, buffer_size, "%.2f/%.2f", tempToDisplay,
               testTempToDisplay);
      temp_1.setText(buffer);

      // logf("[TEMP] Master: %.2f %s | Test: %.2f %s | Diff: %.2f %s\n", tempToDisplay, unit,
      //      testTempToDisplay, unit, tempToDisplay - testTempToDisplay, unit);

      // ============================================================
      // SETPOINT - Actualizar siempre (es una lectura rápida)
      // ============================================================
      float setpointDisplay = settings.getSetpoint(isFahrenheit ? 'F' : 'C');

      // Solo actualizar si cambió (opcional, ahorra comunicación serial)
      if (fabs(setpointDisplay - lastSetpointDisplay) > 0.01f) {
        lastSetpointDisplay = setpointDisplay;
        snprintf(buffer, buffer_size, "%.2f", setpointDisplay);
        setpoint_1.setText(buffer);
      }
    }

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
  // 1. Filtrado de seguridad (Solo procesamos si empieza con $)
  if (command[0] != '$') {
    return; // Ignora logs y basura silenciosamente
  }

  // 2. Si pasó el filtro, nos saltamos el '$' para procesar
  // Esto hace que el resto de tu código NO necesite cambios
  const char *realCommand = command + 1;

  // Copia local modificable
  char cmd[CMD_BUFFER_SIZE];
  strncpy(cmd, realCommand, CMD_BUFFER_SIZE);
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

    if (!param) {
      logln("Invalid SET command. Use $SET <PARAM> <VALUE>");
      return;
    }

    // -------- PID --------
    if (strcmp(param, "KP") == 0) {
      char *value = strtok_r(nullptr, " ", &saveptr);
      if (value && strlen(value) > 0) {
        float oldKp = settings.getPidKp();
        settings.setPidKp(atof(value));
        logf("Old KP: %.2f | New KP set to: %.2f\n", oldKp,
             settings.getPidKp());
      } else {
        logln("Usage: $SET KP <value>");
        logln("  Example: $SET KP 1.5");
      }

    } else if (strcmp(param, "TI") == 0) {
      char *value = strtok_r(nullptr, " ", &saveptr);
      if (value && strlen(value) > 0) {
        float oldTi = settings.getPidTi();
        settings.setPidTi(atof(value));
        logf("Old TI: %.2f | New TI set to: %.2f\n", oldTi,
             settings.getPidTi());
      } else {
        logln("Usage: $SET TI <value>");
        logln("  Example: $SET TI 10");
      }

    } else if (strcmp(param, "TD") == 0) {
      char *value = strtok_r(nullptr, " ", &saveptr);
      if (value && strlen(value) > 0) {
        float oldTd = settings.getPidTd();
        settings.setPidTd(atof(value));
        logf("Old TD: %.2f | New TD set to: %.2f\n", oldTd,
             settings.getPidTd());
      } else {
        logln("Usage: $SET TD <value>");
        logln("  Example: $SET TD 0.5");
      }

    } else if (strcmp(param, "SETPOINT") == 0) {
      char *valueStr = strtok_r(nullptr, " ", &saveptr);
      if (valueStr && strlen(valueStr) > 0) {
        float value;
        char unit;
        parseTempAndUnit(valueStr, value, unit);
        settings.setSetpoint(value, unit);
        logf("SETPOINT set to: %.2f°C / %.2f°F\n", settings.getSetpoint('C'),
             settings.getSetpoint('F'));
        renderConfigUI(0);
      } else {
        logln("Usage: $SET SETPOINT <temp>[unit]");
        logln("  Example: $SET SETPOINT 27C");
        logln("  Example: $SET SETPOINT 80.6F");
      }
      // -------- OFFSETS --------
    } else if (strcmp(param, "MASTER_CORR") == 0) {
      char *indexStr = strtok_r(nullptr, " ", &saveptr);
      char *valueStr = strtok_r(nullptr, " ", &saveptr);

      if (!indexStr || !valueStr || strlen(valueStr) == 0) {
        logln("Usage: $SET MASTER_CORR <1|2|3> <temp>[unit]");
        logln("  Example: $SET MASTER_CORR 1 -0.5C");
        logln("  Example: $SET MASTER_CORR 2 0.9F");
        return;
      }

      int index = atoi(indexStr) - 1;
      float value;
      char unit;

      if (index >= 0 && index < N_POINTS) {
        parseTempAndUnit(valueStr, value, unit);
        settings.setMasterCorrections(index, value, unit);
        settings.save();

        logf("Master correction[%d] set to %.3f°C / %.3f°F (point %.1f°C / "
             "%.1f°F)\n",
             index + 1, settings.getMasterCorrections(index, 'C'),
             settings.getMasterCorrections(index, 'F'),
             settings.getCalibrationPoint(index, 'C'),
             settings.getCalibrationPoint(index, 'F'));
      } else {
        logln("Index must be 1...");
      }

    } else if (strcmp(param, "TEST_CORR") == 0) {
      char *indexStr = strtok_r(nullptr, " ", &saveptr);
      char *valueStr = strtok_r(nullptr, " ", &saveptr);

      if (!indexStr || !valueStr || strlen(valueStr) == 0) {
        logln("Usage: $SET TEST_CORR <1|2|3> <temp>[unit]");
        logln("  Example: $SET TEST_CORR 1 -0.5C");
        logln("  Example: $SET TEST_CORR 2 0.9F");
        return;
      }

      int index = atoi(indexStr) - 1;
      float value;
      char unit;

      if (index >= 0 && index < N_POINTS) {
        parseTempAndUnit(valueStr, value, unit);
        settings.setTestCorrections(index, value, unit);
        settings.save();

        logf("Test correction[%d] set to %.3f°C / %.3f°F (point %.1f°C / "
             "%.1f°F)\n",
             index + 1, settings.getTestCorrections(index, 'C'),
             settings.getTestCorrections(index, 'F'),
             settings.getCalibrationPoint(index, 'C'),
             settings.getCalibrationPoint(index, 'F'));
      } else {
        logln("Index must be 1...");
      }

      // -------- CALIB POINTS --------
    } else if (strcmp(param, "CAL_POINT") == 0) {
      char *indexStr = strtok_r(nullptr, " ", &saveptr);
      char *valueStr = strtok_r(nullptr, " ", &saveptr);

      if (!indexStr || !valueStr || strlen(valueStr) == 0) {
        logln("Usage: $SET CAL_POINT <1|2|3> <temperature>");
        logln("  Example: $SET CAL_POINT 1 -0.5C");
        logln("  Example: $SET CAL_POINT 2 0.9F");
        return;
      }

      int index = atoi(indexStr) - 1;
      float value;
      char unit;

      if (index >= 0 && index < 3) {
        parseTempAndUnit(valueStr, value, unit);
        settings.setCalibrationPoint(index, value, unit);
        settings.save();

        logf("Calibration point[%d] set to %.1f°C / %.1f°F\n", index + 1,
             settings.getCalibrationPoint(index, 'C'),
             settings.getCalibrationPoint(index, 'F'));
      } else {
        logln("Index must be 1, 2, or 3");
      }

      // -------- TOLERANCE --------
    } else if (strcmp(param, "TOLERANCE") == 0) {
      char *valueStr = strtok_r(nullptr, " ", &saveptr);
      if (valueStr && strlen(valueStr) > 0) {
        float value;
        char unit;
        parseTempAndUnit(valueStr, value, unit);
        float oldTolC = settings.getCalibrationTolerance('C');
        float oldTolF = settings.getCalibrationTolerance('F');
        settings.setCalibrationTolerance(value, unit);
        settings.save();
        logf(
            "Old Tolerance: %.3f°C / %.3f°F | New Tolerance: %.3f°C / %.3f°F\n",
            oldTolC, oldTolF, settings.getCalibrationTolerance('C'),
            settings.getCalibrationTolerance('F'));
      } else {
        logln("Usage: $SET TOLERANCE <temp>[unit]");
        logln("  Example: $SET TOLERANCE 0.5C");
        logln("  Example: $SET TOLERANCE 0.9F");
      }

      // -------- LIMITES --------
    } else if (strcmp(param, "HIGH") == 0) {
      char *valueStr = strtok_r(nullptr, " ", &saveptr);
      if (valueStr && strlen(valueStr) > 0) {
        float value;
        char unit;
        parseTempAndUnit(valueStr, value, unit);
        float oldUpperC = settings.getAlarmUpperLimit('C');
        float oldUpperF = settings.getAlarmUpperLimit('F');
        settings.setAlarmUpperLimit(value, unit);
        settings.save();
        logf("Old Alarm High: %.3f°C / %.3f°F | New Alarm High set to: %.3f°C "
             "/ %.3f°F\n",
             oldUpperC, oldUpperF, settings.getAlarmUpperLimit('C'),
             settings.getAlarmUpperLimit('F'));
      } else {
        logln("Usage: $SET HIGH <temp>[unit]");
        logln("  Example: $SET HIGH 80.6F");
        logln("  Example: $SET HIGH 27C");
      }

    } else if (strcmp(param, "LOW") == 0) {
      char *valueStr = strtok_r(nullptr, " ", &saveptr);
      if (valueStr && strlen(valueStr) > 0) {
        float value;
        char unit;
        parseTempAndUnit(valueStr, value, unit);
        float oldLowerC = settings.getAlarmLowerLimit('C');
        float oldLowerF = settings.getAlarmLowerLimit('F');
        settings.setAlarmLowerLimit(value, unit);
        settings.save();
        logf("Old Alarm Low: %.3f°C / %.3f°F | New Alarm Low set to: %.3f°C / "
             "%.3f°F\n",
             oldLowerC, oldLowerF, settings.getAlarmLowerLimit('C'),
             settings.getAlarmLowerLimit('F'));
      } else {
        logln("Usage: $SET LOW <temp>[unit]");
        logln("  Example: $SET LOW 70.2F");
        logln("  Example: $SET LOW 21.2C");
      }

    } else if (strcmp(param, "DANGER") == 0) {
      char *valueStr = strtok_r(nullptr, " ", &saveptr);
      if (valueStr && strlen(valueStr) > 0) {
        float value;
        char unit;
        parseTempAndUnit(valueStr, value, unit);
        float oldDangerC = settings.getDangerTemperature('C');
        float oldDangerF = settings.getDangerTemperature('F');
        settings.setDangerTemperature(value, unit);
        settings.save();
        logf("Old Danger Temp: %.3f°C / %.3f°F | New Danger Temp set to: "
             "%.3f°C / %.3f°F\n",
             oldDangerC, oldDangerF, settings.getDangerTemperature('C'),
             settings.getDangerTemperature('F'));
      } else {
        logln("Usage: $SET DANGER <temp>[unit]");
        logln("  Example: $SET DANGER 90.0F");
        logln("  Example: $SET DANGER 32.2C");
      }

    } else if (strcmp(param, "SAFE") == 0) {
      char *valueStr = strtok_r(nullptr, " ", &saveptr);
      if (valueStr && strlen(valueStr) > 0) {
        float value;
        char unit;
        parseTempAndUnit(valueStr, value, unit);
        float oldSafeC = settings.getSafeTemperature('C');
        float oldSafeF = settings.getSafeTemperature('F');
        settings.setSafeTemperature(value, unit);
        settings.save();
        logf("Old Safe Temp: %.3f°C / %.3f°F | New Safe Temp set to: %.3f°C / "
             "%.3f°F\n",
             oldSafeC, oldSafeF, settings.getSafeTemperature('C'),
             settings.getSafeTemperature('F'));
      } else {
        logln("Usage: $SET SAFE <temp>[unit]");
        logln("  Example: $SET SAFE 85.0F");
        logln("  Example: $SET SAFE 29.4C");
      }

      // -------- TIME --------
    } else if (strcmp(param, "PERIOD") == 0) {
      char *value = strtok_r(nullptr, " ", &saveptr);
      if (value && strlen(value) > 0) {
        int oldPeriod = settings.getPidPeriod();
        int v = atoi(value);
        if (v > 0) {
          settings.setPidPeriod(v);
          settings.save();
          logf("Old PID Period: %d seconds | New PID Period set to: %d "
               "seconds\n",
               oldPeriod, v);
        } else {
          logln("ERROR: PERIOD must be > 0");
        }
      } else {
        logln("Usage: $SET PERIOD <value>");
        logln("  Example: $SET PERIOD 1");
      }

    } else if (strcmp(param, "STABLE") == 0) {
      char *value = strtok_r(nullptr, " ", &saveptr);
      if (value && strlen(value) > 0) {
        int oldStable = settings.getStabilityTime();
        int v = atoi(value);
        if (v >= 0) {
          settings.setStabilityTime(v);
          settings.save();
          logf("Old Stability Time: %d seconds | New Stability Time set to: %d "
               "seconds\n",
               oldStable, v);
        } else {
          logln("ERROR: STABLE must be >= 0");
        }
      } else {
        logln("Usage: $SET STABLE <value>");
        logln("  Example: $SET STABLE 120");
      }

    } else if (strcmp(param, "WAITPROCESS") == 0) {
      char *value = strtok_r(nullptr, " ", &saveptr);
      if (value && strlen(value) > 0) {
        float oldWait = settings.getMaxProcessTime();
        float v = atof(value);
        if (v > 0) {
          settings.setMaxProcessTime(v);
          settings.save();
          logf("Old Max Process Time: %.0f seconds | New Max Process Time set "
               "to: %.0f "
               "seconds\n",
               oldWait, v);
        } else {
          logln("ERROR: WAITPROCESS must be > 0");
        }
      } else {
        logln("Usage: $SET WAITPROCESS <value>");
        logln("  Example: $SET WAITPROCESS 600");
      }

      // -------- ENUMS --------
    } else if (strcmp(param, "SCALE") == 0) {
      char *value = strtok_r(nullptr, " ", &saveptr);
      if (value && strlen(value) > 0) {
        if (strcmp(value, "C") == 0) {
          settings.setTemperatureScale(TemperatureScale::CELSIUS);

          settings.save();
          logln("Scale set to Celsius");
        } else if (strcmp(value, "F") == 0) {
          settings.setTemperatureScale(TemperatureScale::FAHRENHEIT);
          settings.save();
          logln("Scale set to Fahrenheit");
        } else {
          logln("ERROR: SCALE value must be C or F");
        }
      } else {
        logln("Usage: $SET SCALE <C|F>");
        logln("  Example: $SET SCALE C");
        logln("  Example: $SET SCALE F");
      }

      // -------- SENSORS --------
    } else if (strcmp(param, "SENSOR") == 0) {
      char *value = strtok_r(nullptr, " ", &saveptr);
      if (value && strlen(value) > 0) {
        if (strcmp(value, "100") == 0) {
          sensors.configureTestSensor(SensorType::PT100,
                                      settings.getSensorWires());
          settings.save();
          logln("Sensor set to PT100");

          settings.save();
        } else if (strcmp(value, "1000") == 0) {
          sensors.configureTestSensor(SensorType::PT1000,
                                      settings.getSensorWires());
          settings.save();
          logln("Sensor set to PT1000");
        } else {
          logln("ERROR: SENSOR value must be 100 or 1000");
        }
      } else {
        logln("Usage: $SET SENSOR <100|1000>");
        logln("  Example: $SET SENSOR 100");
        logln("  Example: $SET SENSOR 1000");
      }

    } else if (strcmp(param, "WIRES") == 0) {
      char *value = strtok_r(nullptr, " ", &saveptr);
      if (value && strlen(value) > 0) {
        int w = atoi(value);
        if (w >= 2 && w <= 4) {
          sensors.configureTestSensor(settings.getSensorType(), w);
          settings.save();
          logf("Sensor wires set to: %d\n", w);
        } else {
          logln("ERROR: WIRES value must be 2, 3, or 4");
        }
      } else {
        logln("Usage: $SET WIRES <2|3|4>");
        logln("  Example: $SET WIRES 2");
        logln("  Example: $SET WIRES 3");
      }

    } else if (strcmp(param, "TIME") == 0) {
      char *value = strtok_r(nullptr, " ", &saveptr);
      if (value && strlen(value) > 0) {
        if (timeManager.setDateTimeFromString(value)) {
          settings.save();
          logln("Success: RTC updated.");
        } else {
          logln("ERROR: Invalid date/time format. Use YYYYMMDDHHMM");
        }
      } else {
        logln("Usage: $SET TIME YYYYMMDDHHMM");
        logln("  Example: $SET TIME 202406151430 (June 15, 2024 at 14:30)");
      }

    } else {
      logf("Unknown $SET parameter: %s\n", param);
    }

    return;
  }

  // =================================================
  // ================= COMMANDS ======================
  // =================================================
  if (strcmp(token, "HEAT") == 0 || strcmp(token, "COOL") == 0) {
    startThermalProcess();

  } else if (strcmp(token, "STOP") == 0) {
    stopThermalProcess();
    renderStopButtonUI();
    buzzer.playBlocking(BeepType::STOP);
    logln("All processes stopped.");

  } else if (strcmp(token, "RUN") == 0) {
    startThermalProcess();
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

  } else if (strcmp(token, "TIME") == 0) {
    timeManager.printDateTime();

  } else if (strcmp(token, "RESET_CONFIG") == 0) {
    // Borrar archivo de configuración
    if (LittleFS.remove(CONFIG_FILE)) {
      logln("Config file deleted. Restarting to recreate with defaults...");
      delay(1000);
      ESP.restart();
    } else {
      logln("Failed to delete config file");
    }
  } else if (strcmp(token, "DEMO") == 0) {
    calibration.runDemo();
  } else {
    logf("Unknown command: %s\n", token);
  }
}

void printHelp() {
  logln("Available commands:");
  logln("  $TEMP                            - Show current block temperatures "
        "and setpoint");
  logln("  $SET SETPOINT <temp>[unit]       - Set target temperature");
  logln("  $SET KP <value>                  - Set Kp for PID");
  logln("  $SET TI <value>                  - Set Ti for PID (seconds)");
  logln("  $SET TD <value>                  - Set Td for PID (seconds)");
  logln("  $SET PERIOD <value>              - Set PID cycle period (seconds, "
        "integer)");
  logln("  $SET STABLE <value>              - Set stability time (seconds, "
        "integer)");
  logln("  $SET WAITPROCESS <value>         - Set max process time (seconds, "
        "integer)");
  logln("  $SET HIGH <temp>[unit]                - Set high alarm limit");
  logln("  $SET LOW <temp>[unit]                 - Set low alarm limit");
  logln(
      "  $SET MASTER_CORR <1|2|3> <temp>[unit] - Set master sensor calibration "
      "offset");
  logln("  $SET TEST_CORR <1|2|3> <temp>[unit]   - Set test sensor calibration "
        "offset");
  logln(
      "  $SET DANGER <temp>[unit]              - Set danger temperature limit");
  logln("  $SET SAFE <temp>[unit]                - Set safe temperature limit");
  logln("  $SET CAL_POINT <1|2|3> <temp>[unit]   - Set calibration point 1");
  logln("  $SET SCALE <C|F>                 - Set temperature scale "
        "(Celsius/Fahrenheit)");
  logln("  $SET SENSOR <100|1000>           - Set test sensor type "
        "(PT100/PT1000)");
  logln("  $SET WIRES <2|3|4>               - Set test sensor wiring mode");
  logln("  $SET TIME <YYYYMMDDHHMM>         - Set RTC date and time");
  logln("  $SET TOLERANCE <temp>[unit]      - Set calibration tolerance");
  logln("  --------------------------------------------------");
  logln("  $HEAT                - Start heating (PID control)");
  logln("  $COOL                - Start cooling (PID control)");
  logln("  $STOP                - Stop all processes "
        "(heating/cooling/calibration)");
  logln("  $RUN                 - Start calibration cycle");
  logln(
      "  $DEFAULT             - Reset all settings to default (requires SAVE)");
  logln("  $SAVE                - Save current settings to LittleFS");
  logln("  $SHOW                - Show current settings and temperatures");
  logln("  $TIME                - Show current RTC date and time");
  logln("  $RESET_CONFIG        - Reset configuration to defaults and restart");
  logln("  $HELP                - Show this help message");
}

void printTemperatures() {
  logln("================================================");
  logln("              THERMAL STATUS");
  logln("================================================");

#ifdef SENSOR_SIMULATION
  float master = 25.0f;
  float dutComp = 25.5f;
  float masterFiltered = 25.0f;
  float dutFiltered = 25.5f;
#else
  // Valores SIN filtro (exactos) - SIEMPRE en Celsius
  float master = sensors.getMasterTemperature(); // °C
  float dutComp = sensors.getTestTemperature();  // °C

  // Valores CON filtro (lo que ve la pantalla) - SIEMPRE en Celsius
  float masterFiltered =
      sensors.getFilteredMasterTemperature(0.1f, false);               // °C
  float dutFiltered = sensors.getFilteredTestTemperature(0.1f, false); // °C
#endif

  // Obtener setpoint en °C y °F
  float setpointC = settings.getSetpoint('C');
  float setpointF = settings.getSetpoint('F');

  // Convertir todas las temperaturas a °F
  float masterF = master * 1.8f + 32.0f;
  float dutF = dutComp * 1.8f + 32.0f;
  float masterFilteredF = masterFiltered * 1.8f + 32.0f;
  float dutFilteredF = dutFiltered * 1.8f + 32.0f;
  float differenceC = master - dutComp;
  float differenceF = differenceC * 1.8f;

  // ==================== ENCABEZADO ====================
  logln("");
  logf("  Setpoint:           %.2f °C / %.2f °F\n", setpointC, setpointF);
  logln("----------------------------------------");

  // ==================== TEMPERATURAS ACTUALES ====================
  logf("  MASTER:             %.3f °C / %.3f °F\n", master, masterF);
  // logf("    (filtered):       %.3f °C / %.3f °F\n", masterFiltered,
  //  masterFilteredF);
  logf("  TEST:               %.3f °C / %.3f °F\n", dutComp, dutF);
  // logf("    (filtered):       %.3f °C / %.3f °F\n", dutFiltered,
  // dutFilteredF);
  logf("  Difference:         %+.3f °C / %+.3f °F\n", differenceC, differenceF);
  logln("");

  // ==================== CURVA DE CORRECCIÓN ====================
  logln("  [CORRECTION CURVE - Applied to RAW values]");
  for (int i = 0; i < N_POINTS; i++) {
    // float pointC = 0.0f;
    // float pointF = 0.0f;
    // if (i == 0 || i == 2 || i == 4) {
    //   pointC = settings.getCalibrationPoint(i, 'C');
    //   pointF = settings.getCalibrationPoint(i, 'F');
    // } else if (i == 1 || i == 3) {
    //   float p1 = settings.getCalibrationPoint(i - 1, 'C');
    //   float p2 = settings.getCalibrationPoint(i + 1, 'C');
    //   pointC = p1 + (p2 - p1) * 0.5f;
    //   pointF = pointC * 1.8f + 32.0f;
    // }

    float pointC = settings.getCalibrationPoint(i, 'C');
    float pointF = settings.getCalibrationPoint(i, 'F');
    float masterCorrC = settings.getMasterCorrections(i, 'C');
    float testCorrC = settings.getTestCorrections(i, 'C');

    // Mostrar todo en una línea
    logf("    Point %4.1f°C/%5.1f°F | MASTER: %+6.3f°C/%+6.3f°F | TEST: "
         "%+6.3f°C/%+6.3f°F\n",
         pointC, pointF, masterCorrC, masterCorrC * 1.8f, testCorrC,
         testCorrC * 1.8f);
  }
  logln("");

  // ==================== ESTADO ====================
  if (fabs(differenceC) < 0.2) {
    logln("  STATUS:             ✅ Sensors are well matched");
  } else {
    logf("  STATUS:             ⚠ Sensors differ by %.3f °C / %.3f °F\n",
         differenceC, differenceF);
    logln("  ACTION:             Calibrate both sensors against an external "
          "reference");
  }

  logln("================================================");
}

void printSettings() {
  logln("====================================");
  logln("            SETTINGS");
  logln("====================================");

  bool isFahrenheit =
      (settings.getTemperatureScale() == TemperatureScale::FAHRENHEIT);

  // --- CONTROL & ALARMAS ---
  logln("--- [CONTROL & ALARMS] ---");
  logf("  SETPOINT:       %.2f°C / %.2f°F\n", settings.getSetpoint('C'),
       settings.getSetpoint('F'));
  logf("  Alarm High:     %.2f°C / %.2f°F\n", settings.getAlarmUpperLimit('C'),
       settings.getAlarmUpperLimit('F'));
  logf("  Alarm Low:      %.2f°C / %.2f°F\n", settings.getAlarmLowerLimit('C'),
       settings.getAlarmLowerLimit('F'));
  logf("  Danger Temp.:   %.2f°C / %.2f°F\n",
       settings.getDangerTemperature('C'), settings.getDangerTemperature('F'));
  logf("  Safe Temp.:     %.2f°C / %.2f°F\n", settings.getSafeTemperature('C'),
       settings.getSafeTemperature('F'));

  // --- PID PARAMETERS ---
  logln("--- [PID PARAMETERS] ---");
  logf("  KP:             %.2f\n", settings.getPidKp());
  logf("  TI:             %.2f s\n", settings.getPidTi());
  logf("  TD:             %.2f s\n", settings.getPidTd());
  logf("  PID Period:     %.2f s\n", settings.getPidPeriod());
  logf("  Stability Time: %.2f s\n", settings.getStabilityTime());

  // La tolerancia es un delta, se muestra en la unidad actual
  logf("  Calibration tolerance: %.3f°C / %.3f°F\n",
       settings.getCalibrationTolerance('C'),
       settings.getCalibrationTolerance('F'));
  logf("  Max Process Time: %.2f s\n", settings.getMaxProcessTime());

  // --- SENSOR CONFIG ---
  logln("--- [SENSOR CONFIGURATION] ---");
  logf("  Sensor Type:    %s\n",
       settings.getSensorType() == SensorType::PT100 ? "PT100" : "PT1000");
  logf("  Sensor Wires:   %d\n", settings.getSensorWires());
  logf("  Temp. Scale:    %s\n", isFahrenheit ? "Fahrenheit" : "Celsius");

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
  } else if (sensor == SensorType::PT100) {
    // Si es PT1000, establecer pt_0 como inactivo (false)
    pt_0.setActive(false);
    pt_1.setActive(false);
    pt_2.setActive(false);
    pt_3.setActive(false);
    pt_4.setActive(false);
  }

  TemperatureScale scale = settings.getTemperatureScale();
  if (scale == TemperatureScale::CELSIUS) {
    scale_0.setActive(false);
    scale_1.setActive(false);
    scale_2.setActive(false);
    scale_3.setActive(false);
    scale_4.setActive(false);
  } else if (scale == TemperatureScale::FAHRENHEIT) {
    scale_0.setActive(true);
    scale_1.setActive(true);
    scale_2.setActive(true);
    scale_3.setActive(true);
    scale_4.setActive(true);
  }
}

void renderConfigUI(u_int8_t page) {
  char bufferDefault[16];

  if (page == 0) {
    bool isFahrenheit =
        (settings.getTemperatureScale() == TemperatureScale::FAHRENHEIT);
    float setpointDisplay = settings.getSetpoint(isFahrenheit ? 'F' : 'C');
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f", setpointDisplay);
    setpoint_0.setText(bufferDefault);
  } else if (page == 2) {
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f", settings.getPidKp());
    kp.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f", settings.getPidTi());
    ti.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.2f", settings.getPidTd());
    td.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.1f",
             settings.getPidPeriod());
    period.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.1f",
             settings.getStabilityTime());
    stable.setText(bufferDefault);
  } else if (page == 3) {
    snprintf(bufferDefault, sizeof(bufferDefault), "%.1f",
             settings.getCalibrationPoint(0));
    setp1.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.1f",
             settings.getCalibrationPoint(1));
    setp2.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.1f",
             settings.getCalibrationPoint(2));
    setp3.setText(bufferDefault);
  } else if (page == 4) {
    // snprintf(bufferDefault, sizeof(bufferDefault), "%.2f",
    //          settings.getMasterOffset());
    // moffset.setText(bufferDefault);
    // snprintf(bufferDefault, sizeof(bufferDefault), "%.2f",
    //          settings.getTestOffset());
    // toffset.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.1f",
             settings.getAlarmUpperLimit());
    upperlimit.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.1f",
             settings.getAlarmLowerLimit());
    lowerlimit.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.1f",
             settings.getDangerTemperature());
    danger.setText(bufferDefault);
    snprintf(bufferDefault, sizeof(bufferDefault), "%.1f",
             settings.getSafeTemperature());
    safe.setText(bufferDefault);
  }
}

void startThermalProcess() {
  controlState = ControlState::RUNNING;

  pid.reset();

  pid.setPreviousPV(masterTemp);

  if (!processRunning) {
    processStartTime = millis();
    processRunning = true;
  }

  logf("PID control started.\n");
  logf("[TOLERANCE]: %.2f\n", settings.getCalibrationTolerance());
}

void stopThermalProcess() {
  controlState = ControlState::STOPPED;
  heater.stop();
  calibration.stop();
  fan.stop();
  buzzer.stop();
  pid.reset();
  processRunning = false;
  holdTimerActive = false;
  holdStartTime = 0;
}

void checkAndHandleSetpointStability(float currentTemp, float setpoint) {
  // Solo ejecutar si el proceso está activo
  if (!processRunning || controlState != ControlState::RUNNING) {
    if (holdTimerActive) {
      holdTimerActive = false;
      holdStartTime = 0;
    }
    return;
  }

  // Calcular error respecto al setpoint
  float absError = fabs(setpoint - currentTemp);
  float tolerance = settings.getCalibrationTolerance();

  // Verificar si estamos estables
  bool isStable = (absError <= tolerance);

  if (isStable) {
    // Estamos en zona estable
    if (!holdTimerActive) {
      // Primera vez que entramos en estabilidad
      holdTimerActive = true;
      holdStartTime = millis();
      logf("[STABILITY] Temperature stabilized at %.2f°C (error: %.2f°C). "
           "Holding for %.0f seconds...\n",
           currentTemp, absError, settings.getStabilityTime());
    } else {
      // Verificar si alcanzamos el tiempo requerido
      uint32_t holdElapsed = (millis() - holdStartTime) / 1000;

      // Log de progreso cada 5 segundos
      static uint32_t lastProgressLog = 0;
      if (holdElapsed > 0 && (millis() - lastProgressLog) >= 5000) {
        lastProgressLog = millis();
        logf("[STABILITY] Holding at %.2f°C for %lu/%lu seconds...\n",
             currentTemp, holdElapsed, (uint32_t)settings.getStabilityTime());
      }

      if (holdElapsed >= settings.getStabilityTime()) {
        // Éxito: se mantuvo estable el tiempo requerido
        logf("[STABILITY] ✓ Setpoint reached! Temperature stable at %.2f°C for "
             "%lu seconds. Stopping...\n",
             currentTemp, holdElapsed);

        buzzer.playBlocking(BeepType::SUCCESS);
        stopThermalProcess();
        renderStopButtonUI();
        // Resetear timer
        holdTimerActive = false;
        holdStartTime = 0;
      }
    }
  } else {
    // No estamos estables, resetear el timer
    if (holdTimerActive) {
      logf("[STABILITY] ✗ Stability lost! Error: %.2f°C > tolerance: %.2f°C. "
           "Timer reset.\n",
           absError, tolerance);
      holdTimerActive = false;
      holdStartTime = 0;
    }
  }
}

void renderStopButtonUI() {
  nex.sendCommand((char *)"click stop,1");
  delay(20);
  nex.sendCommand((char *)"click stop,0");
}

void parseTempAndUnit(char *input, float &value, char &unit) {
  // Eliminar espacios
  while (isspace(*input))
    input++;

  // Si el string está vacío
  if (strlen(input) == 0) {
    value = 0.0f;
    unit = 'C';
    return;
  }

  // Buscar la unidad al final
  char *ptr = input;
  while (*ptr && (isdigit(*ptr) || *ptr == '.' || *ptr == '-' || *ptr == '+')) {
    ptr++;
  }

  // Si encontramos un carácter no numérico al final, es la unidad
  if (*ptr != '\0') {
    unit = toupper(*ptr);
    *ptr = '\0'; // Separar el número de la unidad
  } else {
    // No se encontró unidad, usar la escala actual del sistema
    unit = (settings.getTemperatureScale() == TemperatureScale::FAHRENHEIT)
               ? 'F'
               : 'C';
  }

  value = atof(input);

  // Validar que la unidad sea válida
  if (unit != 'C' && unit != 'F') {
    logf("WARNING: Invalid unit '%c', defaulting to Celsius\n", unit);
    unit = 'C';
  }
}
