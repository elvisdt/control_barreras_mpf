#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"


#include <esp_idf_lib_helpers.h>
#include <button.h>

static const char *TAG = "CTRL";

// Definición de pines GPIO 
#define PIN_SENSOR_SA1              GPIO_NUM_4   // Sensor A1 (más alejado)
#define PIN_SENSOR_SA2              GPIO_NUM_18   // Sensor A2 (más cerca)
#define PIN_SENSOR_SB1              GPIO_NUM_19   // Sensor B1 (más cerca)
#define PIN_SENSOR_SB2              GPIO_NUM_21    // Sensor B2 (más alejado)


#define LED_PIN_BLINK GPIO_NUM_2  // Pin del LED integrado en la mayoría de placas ESP32-WROOM
static button_t btn_sa1, btn_sa2, btn_sb1, btn_sb2;


static const char *states[] = {
    [BUTTON_PRESSED]      = "pressed",
    [BUTTON_RELEASED]     = "released",
    [BUTTON_CLICKED]      = "clicked",
    [BUTTON_PRESSED_LONG] = "pressed long",
};


static void on_button_cb(button_t *btn, button_state_t state) {
    const char *TAG_BTN = "BTN";
    //if (state != BUTTON_RELEASED) return;

    ESP_LOGW(TAG_BTN, "[%02d] -> %s", btn->gpio, states[state]);
}


int init_btn_funcs()
{
    // First button connected between GPIO and GND
    // pressed logic level 0, no autorepeat
    btn_sa1.gpio = PIN_SENSOR_SA1;
    btn_sa1.pressed_level = 0;
    btn_sa1.internal_pull = true;
    btn_sa1.autorepeat = false;
    btn_sa1.callback = on_button_cb;

    btn_sa2.gpio = PIN_SENSOR_SA2;
    btn_sa2.pressed_level = 0;
    btn_sa2.internal_pull = true;
    btn_sa2.autorepeat = false;
    btn_sa2.callback = on_button_cb;

    btn_sb1.gpio = PIN_SENSOR_SB1;
    btn_sb1.pressed_level = 0;
    btn_sb1.internal_pull = true;
    btn_sb1.autorepeat = false;
    btn_sb1.callback = on_button_cb;

    btn_sb2.gpio = PIN_SENSOR_SB2;
    btn_sb2.pressed_level = 0;
    btn_sb2.internal_pull = true;
    btn_sb2.autorepeat = false;
    btn_sb2.callback = on_button_cb;

    ESP_ERROR_CHECK(button_init(&btn_sa1));
    ESP_ERROR_CHECK(button_init(&btn_sa2));
    ESP_ERROR_CHECK(button_init(&btn_sb1));
    ESP_ERROR_CHECK(button_init(&btn_sb2));
	return 0;
}



void app_main() {
    ESP_LOGI(TAG, "Iniciando sistema de control vehicular y peatonal con interrupciones");
    
    init_btn_funcs();
    ESP_LOGI(TAG, "Sistema inicializado correctamente - Usando interrupciones GPIO");
}
