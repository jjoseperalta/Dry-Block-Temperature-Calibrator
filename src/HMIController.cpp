// HMIController.cpp
#include "HMIController.h"
#include <Arduino.h> // Incluir para usar Serial, etc.

// Definición de objetos Nextion (NO SON extern aquí)
Nextion nex(Serial2);

NextionPage page0(nex, 0, 0, "page0");
NextionDualStateButton pt_0(nex, 0, 1, "bt0");
NextionDualStateButton scale_0(nex, 0, 2, "bt1");
NextionPicture danger_0(nex, 0, 3, "p1");
NextionButton quickTest(nex, 0, 4, "quicktest");
NextionButton advancedTest(nex, 0, 5, "advancedtest");

NextionPage page1(nex, 1, 0, "page1");
NextionDualStateButton pt_1(nex, 1, 1, "bt0");
NextionDualStateButton scale_1(nex, 1, 2, "bt1");
NextionPicture danger_1(nex, 1, 3, "p1");
NextionText temp_1(nex, 1, 6, "temp");
NextionText setp(nex, 1, 7, "setp");
NextionDualStateButton heat(nex, 1, 8, "heat");
NextionDualStateButton cool(nex, 1, 9, "cool");
NextionButton stop_1(nex, 1, 10, "stop");
NextionButton home_1(nex, 1, 11, "home");

NextionPage page2(nex, 2, 0, "page2");
NextionDualStateButton pt_2(nex, 2, 1, "bt0");
NextionDualStateButton scale_2(nex, 2, 2, "bt1");
NextionPicture danger_2(nex, 2, 3, "p1");
NextionText temp_2(nex, 2, 12, "temp");
NextionText time_2(nex, 2, 13, "time");
//NextionText
NextionDualStateButton run(nex, 2, 42, "run");
NextionButton config(nex, 2, 43, "config");
NextionButton stop_2(nex, 2, 44, "stop");
NextionButton home_2(nex, 2, 45, "home");

NextionPage page3(nex, 3, 0, "page3");
NextionDualStateButton pt_3(nex, 3, 1, "bt0");
NextionDualStateButton scale_3(nex, 3, 2, "bt1");
NextionPicture danger_3(nex, 3, 3, "p1");
NextionText kp(nex, 3, 46, "kp");
NextionText ti(nex, 3, 47, "ti");
NextionText td(nex, 3, 48, "td");
NextionText period(nex, 3, 49, "period");
NextionText stability(nex, 3, 50, "stable");
NextionButton next_3(nex, 3, 51, "next");
NextionButton default_3(nex, 3, 52, "default");
NextionButton save_3(nex, 3, 53, "save");
NextionButton backAdvancedTest_3(nex, 3, 54, "backhome");

NextionPage page4(nex, 4, 0, "page4");
NextionDualStateButton pt_4(nex, 4, 1, "bt0");
NextionDualStateButton scale_4(nex, 4, 2, "bt1");
NextionPicture danger_4(nex, 4, 3, "p1");
NextionButton back_4(nex, 4, 55, "back");
NextionText setp1(nex, 4, 56, "setp1");
NextionText setp2(nex, 4, 57, "setp2");
NextionText setp3(nex, 4, 58, "setp3");
NextionText setp4(nex, 4, 59, "setp4");
NextionButton next_4(nex, 4, 60, "next");
NextionButton default_4(nex, 4, 61, "default");
NextionButton save_4(nex, 4, 62, "save");
NextionButton backAdvancedTest_4(nex, 4, 63, "backhome");

NextionPage page5(nex, 5, 0, "page5");
NextionDualStateButton pt_5(nex, 5, 1, "bt0");
NextionDualStateButton scale_5(nex, 5, 2, "bt1");
NextionPicture danger_5(nex, 5, 3, "p1");
NextionButton back_5(nex, 5, 64, "back");
NextionText moffset(nex, 5, 65, "moffset");
NextionText toffset(nex, 5, 66, "toffset");
NextionText upperlimit(nex, 5, 67, "upperlimit");
NextionText lowerlimit(nex, 5, 68, "lowerlimit");
NextionText danger(nex, 5, 69, "danger");
NextionText safe(nex, 5, 70, "safe");
NextionButton default_5(nex, 5, 71, "default");
NextionButton save_5(nex, 5, 72, "save");
NextionButton backAdvancedTest_5(nex, 5, 73, "backhome");

// Definición del mapa estático (debe ser inicializado)
std::map<uint8_t, NextionCallbackFunc> HMIController::eventMap;

void HMIController::init() {
    // Inicializar puerto Serial para Nextion (Serial2 en ESP32)
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    // Inicializar la librería Nextion
    nex.init();

    // Adjuntar la función de callback (PUNTERO DE FUNCIÓN PASADO DESDE *.INO)
    pt_0.attachCallback(&HMIController::globalNextionCallback);
}

void HMIController::poll() {
    // Solo hacemos el polling de Nextion
    nex.poll();
}

// Lógica del Gestor de Eventos
void HMIController::globalNextionCallback(NextionEventType type, INextionTouchable *widget) {
    // Intentamos buscar el ID del componente que generó el evento en el mapa
    uint8_t componentId = widget->getComponentID();
    
    if (eventMap.count(componentId)) {
        // Si el ID está en el mapa, llamamos a la función registrada (callback1, callback2, etc.)
        eventMap[componentId](type, widget);
    } else {
        // Opcional: Manejo de eventos no mapeados
        Serial.print("Evento del componente ID ");
        Serial.print(componentId);
        Serial.println(" recibido, pero no tiene callback registrado.");
    }
}

// Función para registrar los callbacks
bool HMIController::registerCallback(INextionTouchable* componentPtr, NextionCallbackFunc callbackPtr) {
    if (componentPtr == nullptr || callbackPtr == nullptr) {
        Serial.println("ERROR: Puntero de componente o callback nulo.");
        return false;
    }
    
    // 1. Obtener el ID del componente directamente del objeto
    uint8_t componentId = componentPtr->getComponentID();
    
    // 2. Registrar el ID y el callback en nuestro mapa
    eventMap[componentId] = callbackPtr;
    
    // 3. Asegurarse de que el callback global esté adjunto al widget
    componentPtr->attachCallback(&HMIController::globalNextionCallback);
    
    Serial.print("Registrado: Componente con ID ");
    Serial.print(componentId);
    Serial.println(" -> OK.");
    return true;
}