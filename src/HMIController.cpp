// HMIController.cpp
#include "HMIController.h"
#include <Arduino.h>
#include <map>
#include "Logger.h"

// Definición de objetos Nextion (NO SON extern aquí)
Nextion nex(Serial2);

NextionPage page0(nex, 0, 0, "page0");
NextionDualStateButton pt_0(nex, 0, 1, "bt0");
NextionDualStateButton scale_0(nex, 0, 2, "bt1");
NextionText date_0(nex, 0, 3, "t0");
NextionText time_0(nex, 0, 4, "t1");
NextionText temp_0(nex, 0, 5, "temp");
NextionText test_0(nex, 0, 6, "test");
NextionText setpoint_0(nex, 0, 7, "setpoint");
NextionDualStateButton set(nex, 0, 8, "set");
NextionDualStateButton run_0(nex, 0, 9, "run");
NextionButton stop_0(nex, 0, 10, "stop");
NextionButton config_0(nex, 0, 11, "config");

NextionPage page1(nex, 1, 0, "page1");
NextionDualStateButton pt_1(nex, 1, 1, "bt0");
NextionDualStateButton scale_1(nex, 1, 2, "bt1");
NextionText date_1(nex, 1, 3, "t0");
NextionText time_1(nex, 1, 4, "t1");
NextionText client_1(nex, 1, 5, "client");
NextionText temp_1(nex, 1, 6, "temp");
NextionText setpoint_1(nex, 1, 7, "setpoint");

NextionText temp_p1(nex, 1, 12, "t2");
NextionText master_p1(nex, 1, 13, "t3");
NextionText test_p1(nex, 1, 14, "t4");
NextionText diff_p1(nex, 1, 15, "t5");
NextionText dir_p1(nex, 1, 16, "t6");

NextionText temp_p2(nex, 1, 17, "t7");
NextionText master_p2(nex, 1, 18, "t8");
NextionText test_p2(nex, 1, 19, "t9");
NextionText diff_p2(nex, 1, 20, "t10");
NextionText dir_p2(nex, 1, 21, "t11");

NextionText temp_p3(nex, 1, 22, "t12");
NextionText master_p3(nex, 1, 23, "t13");
NextionText test_p3(nex, 1, 24, "t14");
NextionText diff_p3(nex, 1, 25, "t15");
NextionText dir_p3(nex, 1, 26, "t16");

NextionText temp_p4(nex, 1, 27, "t17");
NextionText master_p4(nex, 1, 28, "t18");
NextionText test_p4(nex, 1, 29, "t19");
NextionText diff_p4(nex, 1, 30, "t20");
NextionText dir_p4(nex, 1, 31, "t21");

NextionText temp_p5(nex, 1, 32, "t22");
NextionText master_p5(nex, 1, 33, "t23");
NextionText test_p5(nex, 1, 34, "t24");
NextionText diff_p5(nex, 1, 35, "t25");
NextionText dir_p5(nex, 1, 36, "t26");

NextionDualStateButton run_1(nex, 1, 37, "run");
NextionButton config_1(nex, 1, 38, "config");
NextionButton stop_1(nex, 1, 39, "stop");
NextionButton backhome_1(nex, 1, 40, "backhome");

NextionPage page2(nex, 2, 0, "page2");
NextionDualStateButton pt_2(nex, 2, 1, "bt0");
NextionDualStateButton scale_2(nex, 2, 2, "bt1");
NextionText date_2(nex, 2, 3, "t0");
NextionText time_2(nex, 2, 4, "t1");
NextionText kp(nex, 2, 41, "kp");
NextionText ti(nex, 2, 42, "ti");
NextionText td(nex, 2, 43, "td");
NextionText period(nex, 2, 44, "period");
NextionText stable(nex, 2, 45, "stable");
NextionButton next_2(nex, 2, 46, "next");
NextionButton default_2(nex, 2, 47, "default");
NextionButton save_2(nex, 2, 48, "save");
NextionButton backhome_2(nex, 2, 49, "backhome");

NextionPage page3(nex, 3, 0, "page3");
NextionDualStateButton pt_3(nex, 3, 1, "bt0");
NextionDualStateButton scale_3(nex, 3, 2, "bt1");
NextionText date_3(nex, 3, 3, "t0");
NextionText time_3(nex, 3, 4, "t1");
NextionButton back_3(nex, 3, 50, "back");
NextionText setp1(nex, 3, 51, "setp1");
NextionText setp2(nex, 3, 52, "setp2");
NextionText setp3(nex, 3, 53, "setp3");
NextionButton next_3(nex, 3, 54, "next");
NextionButton default_3(nex, 3, 55, "default");
NextionButton save_3(nex, 3, 56, "save");
NextionButton backhome_3(nex, 3, 57, "backhome");

NextionPage page4(nex, 4, 0, "page4");
NextionDualStateButton pt_4(nex, 4, 1, "bt0");
NextionDualStateButton scale_4(nex, 4, 2, "bt1");
NextionText date_4(nex, 4, 3, "t0");
NextionText time_4(nex, 4, 4, "t1");
NextionButton back_4(nex, 4, 58, "back");
NextionText moffset(nex, 4, 59, "moffset");
NextionText toffset(nex, 4, 60, "toffset");
NextionText upperlimit(nex, 4, 61, "upperlimit");
NextionText lowerlimit(nex, 4, 62, "lowerlimit");
NextionText danger(nex, 4, 63, "danger");
NextionText safe(nex, 4, 64, "safe");
NextionButton default_4(nex, 4, 65, "default");
NextionButton save_4(nex, 4, 66, "save");
NextionButton backhome_4(nex, 4, 67, "backhome");

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
        log("Evento del componente ID ");
        log(componentId);
        logln(" recibido, pero no tiene callback registrado.");
    }
}

// Función para registrar los callbacks
bool HMIController::registerCallback(INextionTouchable* componentPtr, NextionCallbackFunc callbackPtr) {
    if (componentPtr == nullptr || callbackPtr == nullptr) {
        logln("ERROR: Puntero de componente o callback nulo.");
        return false;
    }
    
    // 1. Obtener el ID del componente directamente del objeto
    uint8_t componentId = componentPtr->getComponentID();
    
    // 2. Registrar el ID y el callback en nuestro mapa
    eventMap[componentId] = callbackPtr;
    
    // 3. Asegurarse de que el callback global esté adjunto al widget
    componentPtr->attachCallback(&HMIController::globalNextionCallback);
    
    logf("Registrado: Componente con ID %u -> Callback registrado.\n", componentId);
    return true;
}