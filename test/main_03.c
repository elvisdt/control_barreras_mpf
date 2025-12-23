#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include <inttypes.h>

#include "config_main.h"


static const char *TAG = "GPIO_MONITOR";

// Cola para eventos de GPIO
static QueueHandle_t gpio_evt_queue = NULL;


// Debounce config
#define MAX_GPIO_PINS               GPIO_NUM_MAX
#define DEBOUNCE_TIME_TICKS pdMS_TO_TICKS(100)

// Handler de interrupción con debounce por pin y flanco de bajada
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    static uint32_t last_interrupt_time[MAX_GPIO_PINS] = {0};

    gpio_num_t pin = (gpio_num_t)(int)arg;
    uint32_t now = xTaskGetTickCountFromISR();

    if (pin < MAX_GPIO_PINS) {
        if ((now - last_interrupt_time[pin]) > DEBOUNCE_TIME_TICKS) {
            last_interrupt_time[pin] = now;

            gpio_event_t evt;
            evt.pin = pin;
            evt.state = gpio_get_level(pin);  // debería ser 0 por flanco de bajada

            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(gpio_evt_queue, &evt, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

// Tarea para monitorear eventos de GPIO
static void tarea_monitoreo_gpio(void* arg) {
    gpio_event_t event;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &event, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Evento GPIO - Pin: %d, Estado: %lu", event.pin, event.state);
            if (event.state == 0) {  // Solo flanco de bajada confirmado
                switch (event.pin) {
                    case GPIO_SA1:
                        ESP_LOGI(TAG, "SA1 Activado (flanco de bajada)");
                        break;
                    case GPIO_SA2:
                        ESP_LOGI(TAG, "SA2 Activado (flanco de bajada)");
                        break;
                    case GPIO_SB1:
                        ESP_LOGI(TAG, "SB1 Activado (flanco de bajada)");
                        break;
                    case GPIO_SB2:
                        ESP_LOGI(TAG, "SB2 Activado (flanco de bajada)");
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

void app_main(void) {
    // Configuración de entradas con interrupciones (solo flanco de bajada)
    gpio_config_t input_config = {
        .pin_bit_mask = (1ULL << GPIO_SA1) | (1ULL << GPIO_SA2) |
                        (1ULL << GPIO_SB1) | (1ULL << GPIO_SB2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&input_config);

    // Configuración de salidas
    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << GPIO_OUT_AUX01) | (1ULL << GPIO_OUT_AUX02) |
                        (1ULL << GPIO_B_CERRAR) | (1ULL << GPIO_B_ABRIR) |
                        (1ULL << LED_PIN_BLINK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&output_config);

    // Estado inicial de salidas
    gpio_set_level(GPIO_OUT_AUX01, 0);
    gpio_set_level(GPIO_OUT_AUX02, 0);
    gpio_set_level(GPIO_B_ABRIR, 1);
    gpio_set_level(GPIO_B_CERRAR, 1);
    gpio_set_level(LED_PIN_BLINK, 0);

    // Crear cola para eventos de GPIO
    gpio_evt_queue = xQueueCreate(10, sizeof(gpio_event_t));

    // Instalar servicio de interrupción
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    // Asignar handlers de interrupción para cada GPIO
    gpio_isr_handler_add(GPIO_SA1, gpio_isr_handler, (void*)GPIO_SA1);
    gpio_isr_handler_add(GPIO_SA2, gpio_isr_handler, (void*)GPIO_SA2);
    gpio_isr_handler_add(GPIO_SB1, gpio_isr_handler, (void*)GPIO_SB1);
    gpio_isr_handler_add(GPIO_SB2, gpio_isr_handler, (void*)GPIO_SB2);

    // Crear tarea de monitoreo
    xTaskCreate(tarea_monitoreo_gpio, "gpio_monitor", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Sistema de monitoreo GPIO iniciado");
    uint8_t state = 0;
    // Bucle principal para pruebas de salidas (opcional)
    while (1) {
        gpio_set_level(LED_PIN_BLINK, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_PIN_BLINK, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        state^= 1;
        //gpio_set_level(GPIO_B_ABRIR, state);
        gpio_set_level(GPIO_OUT_AUX01, state);
    }
}
