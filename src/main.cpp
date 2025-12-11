#include <Arduino.h>
#include "Settings.h"
#include "Sensors.h"
#include "Heater.h"
#include "Buzzer.h"
#include "PIDController.h"
#include "Calibration.h"

// Define this to bypass the real sensor reading and use a simulated value
#define SENSOR_SIMULATION

// Function prototypes
void handleCommand(String command);
void printHelp();
void printSettings(); // Mantener el prototipo

// Global objects
Settings settings;
Sensors sensors(settings);
Heater heater;
Buzzer buzzer;
PIDController pid(settings);
Calibration calibration(settings, sensors, buzzer);

// State variables
enum class ControlState {
    STOPPED,
    RUNNING
};

ControlState controlState = ControlState::STOPPED;
unsigned long lastPidTime = 0;
bool targetReachedNotified = false;

void setup() {
    Serial.begin(9600);
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

    buzzer.beep(BeepType::READY);
    Serial.println("Dry Block Temperature Calibrator");
    Serial.println("Type 'HELP' for a list of commands.");
    
    // DEBUG: Imprimimos la configuración inmediatamente después de cargar
    Serial.println("--- DEBUG: Configuración cargada ---");
    printSettings();
    Serial.println("------------------------------------");

    // Initialize PID timer
    lastPidTime = millis();
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
    if (controlState == ControlState::RUNNING && (now - lastPidTime >= (unsigned long)settings.getPidPeriod() * 1000UL)) {
#ifdef SENSOR_SIMULATION
        float masterTemp = 25.0f; // Simulate a constant room temperature
#else
        float masterTemp = sensors.readMasterTemperature();
#endif
        float setpoint = settings.getSetTemperature();
        float dt = (now - lastPidTime) / 1000.0f; // seconds
        float output = pid.calculate(setpoint, masterTemp, dt);
        
        // Un-comment the line below for debugging PID values
        // Serial.println("Output: " + String(output) + " | Temp: " + String(masterTemp) + " | Setpoint: " + String(setpoint));

        if (output >= 0) {
            // We need to heat or maintain temperature
            heater.setCool(0); // Ensure cooling is off
            heater.setHeat(output > 100.0f ? 100.0f : output); // Apply heat, capped at 100
        } else {
            // We need to cool
            heater.setHeat(0); // Ensure heating is off
            heater.setCool(fabsf(output) > 100.0f ? 100.0f : fabsf(output)); // Apply cooling, capped at 100
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
        if (masterTemp > settings.getAlarmUpperLimit() || masterTemp < settings.getAlarmLowerLimit()) {
            buzzer.beep(BeepType::ALARM);
        }

        lastPidTime = now;
    } else if (controlState == ControlState::STOPPED) {
        heater.stop(); // Ensure heater is off when stopped
    }

    // Calibration loop
    calibration.loop();
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
            // Manejar comandos de una palabra (e.g., SETP, SAVE) que deberían haber sido capturados antes, pero por seguridad
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
        } else if (parameter == "MCAL") {
            settings.setMasterCalibrationOffset(valueStr.toFloat());
            Serial.println("MCAL set to: " + String(settings.getMasterCalibrationOffset(), 2));
        } else if (parameter == "HIGH") { // ¡NUEVO!
            settings.setAlarmUpperLimit(valueStr.toFloat());
            Serial.println("Alarm High set to: " + String(settings.getAlarmUpperLimit(), 2));
        } else if (parameter == "LOW") { // ¡NUEVO!
            settings.setAlarmLowerLimit(valueStr.toFloat());
            Serial.println("Alarm Low set to: " + String(settings.getAlarmLowerLimit(), 2));
        } else if (parameter == "PERIOD") { // ¡NUEVO!
            int intValue = valueStr.toInt();
            if (intValue > 0) {
                settings.setPidPeriod(intValue);
                Serial.println("PID Period set to: " + String(settings.getPidPeriod()) + "s");
            } else {
                Serial.println("ERROR: PID Period must be > 0.");
            }
        } else if (parameter == "STABLE") { // ¡NUEVO!
            int intValue = valueStr.toInt();
            if (intValue >= 0) {
                settings.setStabilityTime(intValue);
                Serial.println("Stability Time set to: " + String(settings.getStabilityTime()) + "s");
            } else {
                Serial.println("ERROR: Stability Time must be >= 0.");
            }
        } else if (parameter == "P1") {
            settings.setCalibrationPoint(0, valueStr.toFloat());
            Serial.println("P1 set to: " + String(settings.getCalibrationPoint(0), 2));
        } else if (parameter == "P2") {
            settings.setCalibrationPoint(1, valueStr.toFloat());
            Serial.println("P2 set to: " + String(settings.getCalibrationPoint(1), 2));
        } else if (parameter == "P3") {
            settings.setCalibrationPoint(2, valueStr.toFloat());
            Serial.println("P3 set to: " + String(settings.getCalibrationPoint(2), 2));
        } else if (parameter == "P4") {
            settings.setCalibrationPoint(3, valueStr.toFloat());
            Serial.println("P4 set to: " + String(settings.getCalibrationPoint(3), 2));
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

            // Notifica a la clase Sensors para configurar el hardware (manteniendo los hilos)
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
        Serial.println("PID control started. Target: " + String(settings.getSetTemperature(), 2) + " C");
    } else if (command == "STOP") {
        controlState = ControlState::STOPPED;
        heater.stop();
        calibration.stop();
        Serial.println("All processes stopped.");
    } else if (command == "RUN") {
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
    } else {
        Serial.println("Unknown command");
    }
}

void printHelp() {
    Serial.println("Available commands:");
    Serial.println("  SET SETP <value>    - Set target temperature");
    Serial.println("  SET KP <value>      - Set Kp for PID");
    Serial.println("  SET TI <value>      - Set Ti for PID (seconds)");
    Serial.println("  SET TD <value>      - Set Td for PID (seconds)");
    Serial.println("  SET PERIOD <value>  - Set PID cycle period (seconds, integer) "); 
    Serial.println("  SET STABLE <value>  - Set stability time (seconds, integer)"); 
    Serial.println("  SET HIGH <value>    - Set high alarm limit"); 
    Serial.println("  SET LOW <value>     - Set low alarm limit"); 
    Serial.println("  SET MCAL <value>    - Set master sensor calibration offset");
    Serial.println("  SET P1 <value>      - Set calibration point 1");
    Serial.println("  SET P2 <value>      - Set calibration point 2");
    Serial.println("  SET P3 <value>      - Set calibration point 3");
    Serial.println("  SET P4 <value>      - Set calibration point 4");
    Serial.println("  SET SCALE C|F       - Set temperature scale (Celsius/Fahrenheit)");
    Serial.println("  SET SENSOR 100|1000 - Set test sensor type (PT100/PT1000)");
    Serial.println("  SET WIRES 2|3|4     - Set test sensor wiring mode");
    Serial.println("  --------------------------------------------------");
    Serial.println("  HEAT                - Start heating (PID control)");
    Serial.println("  COOL                - Start cooling (PID control)");
    Serial.println("  STOP                - Stop all processes (heating/cooling/calibration)");
    Serial.println("  RUN                 - Start calibration cycle");
    Serial.println("  DEFAULT             - Reset all settings to default (requires SAVE)");
    Serial.println("  SAVE                - Save current settings to LittleFS");
    Serial.println("  SHOW                - Show current settings and temperatures");
    Serial.println("  HELP                - Show this help message");
}

void printSettings() {
    // Usamos el macro F() para ahorrar RAM y el constructor String(float, decimales) para forzar la precisión.
    Serial.println(F("===================================="));
    Serial.println(F("       CALIBRATOR SETTINGS"));
    Serial.println(F("===================================="));
    
    // --- CONTROL & ALARMAS ---
    Serial.println(F("--- [CONTROL & ALARMAS] ---"));
    Serial.print(F("  SETPOINT: ")); Serial.println(String(settings.getSetTemperature(), 2));
    Serial.print(F("  Alarm High: ")); Serial.println(String(settings.getAlarmUpperLimit(), 2));
    Serial.print(F("  Alarm Low: ")); Serial.println(String(settings.getAlarmLowerLimit(), 2));

    // --- PID PARAMETERS ---
    Serial.println(F("--- [PID PARAMETERS] ---"));
    Serial.print(F("  KP: ")); Serial.println(String(settings.getPidKp(), 2));
    Serial.print(F("  TI: ")); Serial.println(String(settings.getPidTi(), 2));
    Serial.print(F("  TD: ")); Serial.println(String(settings.getPidTd(), 2));
    Serial.print(F("  PID Period: ")); Serial.print(settings.getPidPeriod()); Serial.println(F("s"));
    Serial.print(F("  Stability Time: ")); Serial.print(settings.getStabilityTime()); Serial.println(F("s"));

    // --- SENSOR CONFIG ---
    Serial.println(F("--- [SENSOR CONFIGURATION] ---"));
    String sensorTypeStr = (settings.getSensorType() == SensorType::PT100) ? "PT100" : "PT1000";
    Serial.println("  Sensor Type: " + sensorTypeStr);
    Serial.println("  Sensor Wires: " + String(settings.getSensorWires()));
    String tempScaleStr = (settings.getTemperatureScale() == TemperatureScale::CELSIUS) ? "Celsius" : "Fahrenheit";
    Serial.println("  Temp. Scale: " + tempScaleStr);
    Serial.print(F("  Master Cal. Offset: ")); Serial.println(String(settings.getMasterCalibrationOffset(), 2));
    
    // --- CALIBRATION POINTS ---
    Serial.println(F("--- [CALIBRATION POINTS] ---"));
    Serial.print(F("  Calib. P1: ")); Serial.println(String(settings.getCalibrationPoint(0), 2));
    Serial.print(F("  Calib. P2: ")); Serial.println(String(settings.getCalibrationPoint(1), 2));
    Serial.print(F("  Calib. P3: ")); Serial.println(String(settings.getCalibrationPoint(2), 2));
    Serial.print(F("  Calib. P4: ")); Serial.println(String(settings.getCalibrationPoint(3), 2));

    Serial.println(F("===================================="));
}