/**  @file
  mcu_uart_demo.h

  @brief
  This file is used to define ble gatt demo for different Quectel Project.

*/



#ifndef _CONFIG_MAIN_H_
#define _CONFIG_MAIN_H_


#ifdef __cplusplus
extern "C" {
#endif



#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include <inttypes.h>


/*===================================================================
 *
 *                         MACRO DEFINE
 *
 ====================================================================*/

 
 // Definición de pines GPIO
#define GPIO_SA1          GPIO_NUM_21     // DI_01
#define GPIO_SA2          GPIO_NUM_19     // DI_02
#define GPIO_SB1          GPIO_NUM_18     // DI_03
#define GPIO_SB2          GPIO_NUM_4      // DI_04
#define GPIO_OUT_AUX01    GPIO_NUM_26     // OUT GND 01
#define GPIO_OUT_AUX02    GPIO_NUM_25     // OUT GND 01
#define GPIO_B_ABRIR      GPIO_NUM_22     // Pulso para abrir barreras
#define GPIO_B_CERRAR     GPIO_NUM_23     // Pulso para cerrar barreras
#define LED_PIN_BLINK     GPIO_NUM_2      // LED integrado
 


 // Estructura para eventos de GPIO
typedef struct {
    gpio_num_t pin;
    uint32_t state;
} gpio_event_t;






#ifdef __cplusplus
} /*"C" */
#endif

#endif /* BLE_GATT_DEMO_H */

