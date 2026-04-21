#include "Logger.h"
#include <cstdarg>
#include <cstdio>

// Buffer más grande para líneas largas
static const size_t LOG_BUFFER_SIZE = 1024;
static char logBuffer[LOG_BUFFER_SIZE];

// Configuración de reintentos
static const int MAX_RETRIES = 3;
static const TickType_t MUTEX_TIMEOUT = pdMS_TO_TICKS(500);  // Aumentado a 500ms
static const TickType_t SERIAL_TIMEOUT = pdMS_TO_TICKS(50);   // Timeout para escritura

void logf(const char *format, ...) {
    if (format == nullptr) return;
    
    // Limpiar buffer
    memset(logBuffer, 0, LOG_BUFFER_SIZE);
    
    // Formatear el string
    va_list args;
    va_start(args, format);
    int len = vsnprintf(logBuffer, LOG_BUFFER_SIZE - 1, format, args);
    va_end(args);
    
    if (len < 0 || len >= LOG_BUFFER_SIZE - 1) {
        // Error o buffer muy pequeño, truncar
        logBuffer[LOG_BUFFER_SIZE - 2] = '\n';
        logBuffer[LOG_BUFFER_SIZE - 1] = '\0';
    }
    
    // Verificar que el string termine con newline
    size_t bufLen = strlen(logBuffer);
    if (bufLen > 0 && logBuffer[bufLen - 1] != '\n') {
        strncat(logBuffer, "\n", LOG_BUFFER_SIZE - bufLen - 1);
    }
    
    // Escribir con mutex y reintentos
    if (serialMutex != NULL) {
        int retries = 0;
        while (retries < MAX_RETRIES) {
            if (xSemaphoreTake(serialMutex, MUTEX_TIMEOUT) == pdTRUE) {
                // Verificar que hay espacio en el buffer serial
                size_t available = Serial.availableForWrite();
                size_t toWrite = strlen(logBuffer);
                
                if (available >= toWrite) {
                    // Escribir carácter por carácter con timeout
                    bool success = true;
                    for (size_t i = 0; i < toWrite; i++) {
                        unsigned long startTime = millis();
                        while (Serial.availableForWrite() == 0) {
                            if (millis() - startTime > pdTICKS_TO_MS(SERIAL_TIMEOUT)) {
                                success = false;
                                break;
                            }
                            delay(1);
                        }
                        if (success) {
                            Serial.write(logBuffer[i]);
                        } else {
                            break;
                        }
                    }
                    
                    if (success) {
                        Serial.flush();  // Asegurar que los datos se envían
                    }
                } else {
                    // Buffer lleno, esperar un poco
                    delay(5);
                }
                
                xSemaphoreGive(serialMutex);
                break;  // Salir del bucle de reintentos
            }
            retries++;
            delay(10);  // Esperar antes de reintentar
        }
    } else {
        // Sin mutex, escritura directa (menos segura)
        Serial.print(logBuffer);
        Serial.flush();
    }
}

void log(const char *msg) {
    if (msg == nullptr) return;
    logf("%s", msg);  // Reutilizar logf
}

void logln(const char *msg) {
    if (msg == nullptr) return;
    logf("%s\n", msg);  // Reutilizar logf con newline
}

void log_flush() {
    if (serialMutex != NULL) {
        if (xSemaphoreTake(serialMutex, MUTEX_TIMEOUT) == pdTRUE) {
            Serial.flush();
            xSemaphoreGive(serialMutex);
        }
    } else {
        Serial.flush();
    }
}

void log_begin_transaction() {
    if (serialMutex != NULL) {
        xSemaphoreTake(serialMutex, portMAX_DELAY);  // Esperar indefinidamente
    }
}

void log_end_transaction() {
    if (serialMutex != NULL) {
        Serial.flush();
        xSemaphoreGive(serialMutex);
    }
}