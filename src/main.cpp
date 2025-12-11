#include "Buzzer.h"
#include "Calibration.h"
#include "HMIController.h"
#include "Heater.h"
#include "PIDController.h"
#include "Sensors.h"
#include "Settings.h"
#include <Arduino.h>

// Define this to bypass the real sensor reading and use a simulated value
// #define SENSOR_SIMULATION

// Function prototypes
void handleCommand(String command);
void printHelp();
void printSettings();
void printTemperatures(); // ¡NUEVO! Función para mostrar temperaturas
void renderNavBarUI();
void renderConfigUI(u_int8_t page);

// Global objects
Settings settings;
Sensors sensors(settings);
Heater heater;
Buzzer buzzer;
PIDController pid(settings);
Calibration calibration(settings, sensors, buzzer);
HMIController hmi;

// State variables
enum class ControlState { STOPPED, RUNNING };

ControlState controlState = ControlState::STOPPED;
unsigned long lastPidTime = 0;
bool targetReachedNotified = false;

long contador = 0;
const long intervalo = 500;
unsigned long tiempoAnterior = 0;
char cadenaContador[12];

char buffer[32];
size_t buffer_size = sizeof(buffer);

float masterTemp = 0.0f;

// ----------------------------------------------------
// 1. CAPTURA DEL EVENTO CALLBACK (Definición en el *.ino)
// ----------------------------------------------------
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

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for serial port to connect.
  }

  // Llama a Settings::begin(), que monta LittleFS y llama a settings.load()
  settings.begin();
#ifndef SENSOR_SIMULATION
  sensors.begin();
#endif
  heater.begin();
  buzzer.begin();
  hmi.init();

  buzzer.beep(BeepType::TARGET_REACHED);
  uint32_t t = millis();
while (millis() - t < 600) { // Ejecutar por unos 400ms para asegurar el fin del patrón
    buzzer.handle();
    // Aquí puedes llamar a otras funciones de inicialización no-bloqueantes si las tienes.
}
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
  // HMIController::registerCallback(&run, callbackAdvancedTest);
  HMIController::registerCallback(&config, callbackPage3);
  // HMIController::registerCallback(&stop_2, callbackAdvancedTest);
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
}

void loop() {
  // Read serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    handleCommand(command);
  }

#if 0
    // Old PID timing: uses settings.getPidPeriod() directly as seconds
#endif
  // PID control loop (uses real elapsed time dt)
  unsigned long now = millis();
  masterTemp = sensors.getFilteredMasterTemperature(20);
  float setpoint = settings.getSetTemperature();
  if (controlState == ControlState::RUNNING &&
      (now - lastPidTime >= (unsigned long)settings.getPidPeriod() * 1000UL)) {
#ifdef SENSOR_SIMULATION
    float masterTemp = 25.0f; // Simulate a constant room temperature
#else
    // float masterTemp = sensors.readMasterTemperature();
    // masterTemp = sensors.getFilteredMasterTemperature(10);
#endif
    // float setpoint = settings.getSetTemperature();
    float dt = (now - lastPidTime) / 1000.0f; // seconds
    float output = pid.calculate(setpoint, masterTemp, dt);

    // Un-comment the line below for debugging PID values
    Serial.println("Output: " + String(output) + " | Temp: " +
                   String(masterTemp) + " | Setpoint: " + String(setpoint));

    if (output >= 0) {
      // We need to heat or maintain temperature
      heater.setCool(0); // Ensure cooling is off
      heater.setHeat(output > 100.0f ? 100.0f
                                     : output); // Apply heat, capped at 100
    } else {
      // We need to cool
      heater.setCool(fabsf(output) > 100.0f
                         ? 100.0f
                         : fabsf(output)); // Apply cooling, capped at 100
      heater.setHeat(0);                   // Ensure heating is off
    }

    // // Check if target temperature is reached
    // if (abs(masterTemp - setpoint) < 0.5) {
    //   if (!targetReachedNotified) {
    //     buzzer.beep(BeepType::TARGET_REACHED);
    //     targetReachedNotified = true;
    //     if (calibration.isRunning()) {
    //       calibration.targetReached();
    //     }
    //   }
    // } else {
    //   targetReachedNotified = false;
    // }

    // // Check for alarms
    // if (masterTemp > settings.getAlarmUpperLimit() ||
    //     masterTemp < settings.getAlarmLowerLimit()) {
    //   buzzer.beep(BeepType::ALARM);
    // }

    lastPidTime = now;
  } else if (controlState == ControlState::STOPPED) {
    heater.stop(); // Ensure heater is off when stopped
  }

  // Check if target temperature is reached
    if (abs(masterTemp - setpoint) < 0.5) {
      if (!targetReachedNotified) {
        buzzer.beep(BeepType::TARGET_REACHED);
        targetReachedNotified = true;
        if (calibration.isRunning()) {
          calibration.targetReached();
        }
      }
    } else {
      targetReachedNotified = false;
    }

    // Check for alarms
    if (masterTemp > settings.getAlarmUpperLimit() ||
        masterTemp < settings.getAlarmLowerLimit()) {
      buzzer.beep(BeepType::ALARM);
    }

  // Calibration loop
  calibration.loop();

  hmi.poll();

  buzzer.handle();

  unsigned long tiempoActual = millis();
  if (tiempoActual - tiempoAnterior >= intervalo) {
    tiempoAnterior = tiempoActual;
    contador++;

    const char *unit =
        settings.getTemperatureScale() == TemperatureScale::CELSIUS ? "C" : "F";
    // Convertir el contador a cadena
    //   sprintf(buffer, "%.2f \xB0%s", sensors.readMasterTemperature(), unit);

    // Mostrar en Serial (para depuración)
    //   Serial.print("Temperatura: ");
    //   Serial.println(sensors.readMasterTemperature());

    // snprintf(buffer, buffer_size, "%.2f \xB0%s",
    //          sensors.readMasterTemperature(), unit);
    snprintf(buffer, buffer_size, "%.1f \xB0%s", masterTemp, unit);
    temp_1.setText(buffer);
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

/**
 * @brief Muestra las temperaturas actuales del Master y Test Sensor, junto con
 * el Setpoint.
 * * NOTA: La lectura de sensores es una simulación (#ifdef SENSOR_SIMULATION) o
 * lectura real. Si se usa simulación, la lectura de masterTemp será 25.0f.
 */
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