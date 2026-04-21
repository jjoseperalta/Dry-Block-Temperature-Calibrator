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
extern NextionText date_0;
extern NextionText time_0;
extern NextionText temp_0;
extern NextionText test_0;
extern NextionText setpoint_0;
extern NextionDualStateButton set;
extern NextionDualStateButton run_0;
extern NextionButton stop_0;
extern NextionButton config_0;

extern NextionPage page1;
extern NextionDualStateButton pt_1;
extern NextionDualStateButton scale_1;
extern NextionText date_1;
extern NextionText time_1;
extern NextionText client_1;
extern NextionText temp_1;
extern NextionText setpoint_1;
extern NextionText temp_p1;
extern NextionText master_p1;
extern NextionText test_p1;
extern NextionText diff_p1;
extern NextionText dir_p1;
extern NextionText temp_p2;
extern NextionText master_p2;
extern NextionText test_p2;
extern NextionText diff_p2;
extern NextionText dir_p2;
extern NextionText temp_p3;
extern NextionText master_p3;
extern NextionText test_p3;
extern NextionText diff_p3;
extern NextionText dir_p3;
extern NextionText temp_p4;
extern NextionText master_p4;
extern NextionText test_p4;
extern NextionText diff_p4;
extern NextionText dir_p4;
extern NextionText temp_p5;
extern NextionText master_p5;
extern NextionText test_p5;
extern NextionText diff_p5;
extern NextionText dir_p5;
extern NextionDualStateButton run_1;
extern NextionButton config_1;
extern NextionButton stop_1;
extern NextionButton backhome_1;

extern NextionPage page2;
extern NextionDualStateButton pt_2;
extern NextionDualStateButton scale_2;
extern NextionText date_2;
extern NextionText time_2;
extern NextionText kp;
extern NextionText ti;
extern NextionText td;
extern NextionText period;
extern NextionText stable;
extern NextionButton next_2;
extern NextionButton default_2;
extern NextionButton save_2;
extern NextionButton backhome_2;

extern NextionPage page3;
extern NextionDualStateButton pt_3;
extern NextionDualStateButton scale_3;
extern NextionText date_3;
extern NextionText time_3;
extern NextionButton back_3;
extern NextionText setp1;
extern NextionText setp2;
extern NextionText setp3;
extern NextionButton next_3;
extern NextionButton default_3;
extern NextionButton save_3;
extern NextionButton backhome_3;

extern NextionPage page4;
extern NextionDualStateButton pt_4;
extern NextionDualStateButton scale_4;
extern NextionText date_4;
extern NextionText time_4;
extern NextionButton back_4;
extern NextionText moffset;
extern NextionText toffset;
extern NextionText upperlimit;
extern NextionText lowerlimit;
extern NextionText danger;
extern NextionText safe;
extern NextionButton default_4;
extern NextionButton save_4;
extern NextionButton backhome_4;

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