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
NextionText temp_p1(nex, 2, 14, "t2");
NextionText up_temp_master_p1(nex, 2, 15, "t3");
NextionText up_temp_test_p1(nex, 2, 16, "t4");
NextionText up_diff_p1(nex, 2, 17, "t5");
NextionText temp_p2(nex, 2, 18, "t6");
NextionText up_temp_master_p2(nex, 2, 19, "t7");
NextionText up_temp_test_p2(nex, 2, 20, "t8");
NextionText up_diff_p2(nex, 2, 21, "t9");
NextionText temp_p3(nex, 2, 22, "t10");
NextionText up_temp_master_p3(nex, 2, 23, "t11");
NextionText up_temp_test_p3(nex, 2, 24, "t12");
NextionText up_diff_p3(nex, 2, 25, "t13");
NextionText temp_p4(nex, 2, 26, "t14");
NextionText up_temp_master_p4(nex, 2, 27, "t15");
NextionText up_temp_test_p4(nex, 2, 28, "t16");
NextionText up_diff_p4(nex, 2, 29, "t17");
NextionText down_temp_master_p4(nex, 2, 30, "t18");
NextionText down_temp_test_p4(nex, 2, 31, "t19");
NextionText down_diff_p4(nex, 2, 32, "t20");
NextionText down_temp_master_p3(nex, 2, 33, "t21");
NextionText down_temp_test_p3(nex, 2, 34, "t22");
NextionText down_diff_p3(nex, 2, 35, "t23");
NextionText down_temp_master_p2(nex, 2, 36, "t24");
NextionText down_temp_test_p2(nex, 2, 37, "t25");
NextionText down_diff_p2(nex, 2, 38, "t26");
NextionText down_temp_master_p1(nex, 2, 39, "t27");
NextionText down_temp_test_p1(nex, 2, 40, "t28");
NextionText down_diff_p1(nex, 2, 41, "t29");
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
    
    log("Registrado: Componente con ID ");
    log(componentId);
    logln(" -> OK.");
    return true;
}