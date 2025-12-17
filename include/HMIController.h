// HMIController.h
#ifndef HMICONTROLLER_H
#define HMICONTROLLER_H

#include <Nextion.h>
#include <NextionPage.h>
#include <NextionButton.h>
#include <NextionDualStateButton.h>
#include <NextionPicture.h>
#include <NextionText.h>
#include <NextionNumber.h>
#include <map>

#define RXD2 16
#define TXD2 17

// Declaraciones de objetos Nextion (extern para acceso global desde *.ino)
extern Nextion nex;

extern NextionPage page0;
extern NextionDualStateButton pt_0;
extern NextionDualStateButton scale_0;
extern NextionPicture danger_0;
extern NextionButton quickTest;
extern NextionButton advancedTest;

extern NextionPage page1;
extern NextionDualStateButton pt_1;
extern NextionDualStateButton scale_1;
extern NextionPicture danger_1;
extern NextionText temp_1;
extern NextionText setp;
extern NextionDualStateButton heat;
extern NextionDualStateButton cool;
extern NextionButton stop_1;
extern NextionButton home_1;

extern NextionPage page2;
extern NextionDualStateButton pt_2;
extern NextionDualStateButton scale_2;
extern NextionPicture danger_2;
extern NextionText temp_2;
extern NextionText time_2;
extern NextionText temp_p1;
extern NextionText up_temp_master_p1;
extern NextionText up_temp_test_p1;
extern NextionText up_diff_p1;
extern NextionText temp_p2;
extern NextionText up_temp_master_p2;
extern NextionText up_temp_test_p2;
extern NextionText up_diff_p2;
extern NextionText temp_p3;
extern NextionText up_temp_master_p3;
extern NextionText up_temp_test_p3;
extern NextionText up_diff_p3;
extern NextionText temp_p4;
extern NextionText up_temp_master_p4;
extern NextionText up_temp_test_p4;
extern NextionText up_diff_p4;
extern NextionText down_temp_master_p4;
extern NextionText down_temp_test_p4;
extern NextionText down_diff_p4;
extern NextionText down_temp_master_p3;
extern NextionText down_temp_test_p3;
extern NextionText down_diff_p3;
extern NextionText down_temp_master_p2;
extern NextionText down_temp_test_p2;
extern NextionText down_diff_p2;
extern NextionText down_temp_master_p1;
extern NextionText down_temp_test_p1;
extern NextionText down_diff_p1;
extern NextionDualStateButton run;
extern NextionButton config;
extern NextionButton stop_2;
extern NextionButton home_2;

extern NextionPage page3;
extern NextionDualStateButton pt_3;
extern NextionDualStateButton scale_3;
extern NextionPicture danger_3;
extern NextionText kp;
extern NextionText ti;
extern NextionText td;
extern NextionText period;
extern NextionText stability;
extern NextionButton next_3;
extern NextionButton default_3;
extern NextionButton save_3;
extern NextionButton backAdvancedTest_3;

extern NextionPage page4;
extern NextionDualStateButton pt_4;
extern NextionDualStateButton scale_4;
extern NextionPicture danger_4;
extern NextionButton back_4;
extern NextionText setp1;
extern NextionText setp2;
extern NextionText setp3;
extern NextionText setp4;
extern NextionButton next_4;
extern NextionButton default_4;
extern NextionButton save_4;
extern NextionButton backAdvancedTest_4;

extern NextionPage page5;
extern NextionDualStateButton pt_5;
extern NextionDualStateButton scale_5;
extern NextionPicture danger_5;
extern NextionButton back_5;
extern NextionText moffset;
extern NextionText toffset;
extern NextionText upperlimit;
extern NextionText lowerlimit;
extern NextionText danger;
extern NextionText safe;;
extern NextionButton default_5;
extern NextionButton save_5;
extern NextionButton backAdvancedTest_5;

// Definición del tipo de puntero a función que usaremos en el mapa
typedef void (*NextionCallbackFunc)(NextionEventType, INextionTouchable*);

/**
 * @brief Controlador para la Interfaz Humano-Máquina (HMI) Nextion.
 */
class HMIController {
private:
    // El mapa estático para registrar qué ID de componente llama a qué función
    static std::map<uint8_t, NextionCallbackFunc> eventMap;

    /**
     * @brief El único callback estático que se adjunta a Nextion.
     * Es la puerta de entrada de todos los eventos.
     */
    static void globalNextionCallback(NextionEventType type, INextionTouchable *widget);
public:
    /**
     * @brief Inicializa la comunicación y la librería Nextion.
     */
    static void init();

    /**
     * @brief Función de polling para detectar eventos.
     */
    static void poll();

    /**
     * @brief Método específico para adjuntar un callback al botón 'bt0'.
     * @param callbackPtr Puntero a la función de callback.
     */
    static bool registerCallback(INextionTouchable* componentPtr, NextionCallbackFunc callbackPtr);
};

#endif // HMICONTROLLER_H