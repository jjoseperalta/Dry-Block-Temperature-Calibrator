#include "TimeManager.h"
#include "Logger.h"
#include <Arduino.h>

// Definimos la instancia global aquí
// TimeManager timeManager;

TimeManager::TimeManager() : lastMinute(61) {}

void TimeManager::begin() {
  if (!rtc.begin()) {
    logln("ERROR: No se pudo encontrar el modulo RTC DS3231");
    return;
  }

  if (rtc.lostPower()) {
    logln("RTC perdio alimentacion, ajustando a la fecha de compilacion...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

bool TimeManager::update() {
  DateTime now = rtc.now();

  // Si el minuto actual es distinto al guardado, actualizamos
  if (now.minute() != lastMinute) {
    lastMinute = now.minute();
    updateHMI(); // Llamamos a la actualización interna de la pantalla
    return true;
  }

  return false;
}

void TimeManager::updateHMI() {
  DateTime now = rtc.now();
  char timeBuf[6];  // HH:mm
  char dateBuf[11]; // DD/MM/YYYY

  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", now.hour(), now.minute());
  snprintf(dateBuf, sizeof(dateBuf), "%02d/%02d/%d", now.day(), now.month(),
           now.year());

  // 1. Creamos arreglos de punteros a los objetos globales existentes
  NextionText *timeComponents[] = {&time_0, &time_1, &time_2, &time_3, &time_4};
  NextionText *dateComponents[] = {&date_0, &date_1, &date_2, &date_3, &date_4};

  // 2. Iteramos y actualizamos todos de un solo golpe
  // Usar i < 5 o mejor aún:
  for (int i = 0; i < (sizeof(timeComponents) / sizeof(timeComponents[0]));
       i++) {
    timeComponents[i]->setText(timeBuf);
    dateComponents[i]->setText(dateBuf);
  }

  // logf("TimeManager: Hora actualizada a %s\n", timeBuf);
  // logf("TimeManager: Fecha actualizada a %s\n", dateBuf);
}

String TimeManager::getFormattedTime() {
  DateTime now = rtc.now();
  char buf[9];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", now.hour(), now.minute(),
           now.second());
  return String(buf);
}

bool TimeManager::setDateTimeFromString(const char* dateTimeStr) {
  // 1. Validar longitud exacta: YYYYMMDDHHMM son exactamente 12 caracteres
  if (strlen(dateTimeStr) != 12) {
    // logln("RTC Error: String length must be exactly 12 characters.");
    return false;
  }

  // 2. Validar que todos los caracteres sean números
  for (uint8_t i = 0; i < 12; i++) {
    if (!isdigit(dateTimeStr[i])) {
      // logln("RTC Error: String contains non-numeric characters.");
      return false;
    }
  }

  // Variables temporales optimizadas en memoria para el ESP32
  uint16_t year;
  uint8_t month, day, hour, minute;

  // 3. Parsear el string aplicando máscaras de ancho fijo (%4hd para 4 dígitos, %2hhd para 2 dígitos)
  // sscanf devuelve la cantidad de variables asignadas exitosamente. Esperamos 5.
  if (sscanf(dateTimeStr, "%4hd%2hhd%2hhd%2hhd%2hhd", &year, &month, &day, &hour, &minute) != 5) {
    return false;
  }

  // 4. Validaciones de rangos lógicos
  if (year < 2000 || year > 2099) return false; // El DS3231 maneja el rango 2000-2099
  if (month < 1 || month > 12)    return false;
  if (day < 1 || day > 31)        return false;
  if (hour > 23)                  return false;
  if (minute > 59)                return false;

  // 4.1 Validación específica de días por mes (Bisiestos incluidos de forma simple)
  uint8_t daysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  // Si es bisiesto, Febrero tiene 29 días
  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
    daysInMonth[1] = 29;
  }
  if (day > daysInMonth[month - 1]) {
    // logln("RTC Error: Invalid day for the specified month.");
    return false;
  }

  // 5. Si pasó todas las validaciones, actualizamos el RTC físicamente
  rtc.adjust(DateTime(year, month, day, hour, minute, 0));

  // 6. Forzar actualización inmediata en la pantalla Nextion en el siguiente ciclo
  lastMinute = 61; 

  // Log de confirmación estructurado
  logf("RTC Success: Updated to %02d/%02d/%04d %02d:%02d:00\n", day, month, year, hour, minute);
  
  return true;
}

void TimeManager::printDateTime() {
  DateTime now = rtc.now();
  if (now.isValid()) {
    char buf[20];
    snprintf(buf, sizeof(buf), "%02d/%02d/%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
    logf("RTC Current DateTime: %s\n", buf);
  } else {
    logln("RTC Error: Invalid DateTime read from RTC.");
  }
}