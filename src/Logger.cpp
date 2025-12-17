#include "Logger.h"
#include <cstdarg>
#include <cstdio>

void logf(const char* format, ...) {
    if (!serialMutex) return;

    char buffer[256];   // suficiente para logs normales

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.print(buffer);
        xSemaphoreGive(serialMutex);
    }
}

void log(const char* msg) {
    if (!serialMutex) return;

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.print(msg);
        xSemaphoreGive(serialMutex);
    }
}

void logln(const char* msg) {
    if (!serialMutex) return;

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.println(msg);
        xSemaphoreGive(serialMutex);
    }
}
