#ifndef TIMEMANAGER_H
#define TIMEMANAGER_H

#include <RTClib.h>
#include "HMIController.h"

class TimeManager {
public:
    TimeManager();
    
    // Inicializa el RTC y ajusta la hora si es necesario
    void begin();

    // Esta función se llama en el loop o tarea de interfaz.
    // Retorna 'true' solo en el instante que el minuto cambia.
    bool update();
    
    void updateHMI();

    // Obtener strings formateados si los necesitas para otra cosa
    String getFormattedTime();
    String getFormattedDate();

    bool setDateTimeFromString(const char* dateTimeStr);

    void printDateTime();

private:
    RTC_DS3231 rtc;
    uint8_t lastMinute;
};

// extern TimeManager timeManager;

#endif