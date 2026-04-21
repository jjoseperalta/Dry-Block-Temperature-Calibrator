#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern SemaphoreHandle_t serialMutex;

// printf-style
void logf(const char* format, ...);

// print-style
void log(const char* msg);

// println-style
void logln(const char* msg);

// Nuevas funciones para asegurar integridad
void log_flush();
void log_begin_transaction();
void log_end_transaction();