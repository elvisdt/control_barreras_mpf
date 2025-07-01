// control_barreras_fsm.c
// Arquitectura basada en tareas separadas para control de sensores y actuadores

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "button.h"

#define TAG "FSM_BARRERA"

// Pines de sensores, barreras y alarma
#define SA1 GPIO_NUM_4
#define SA2 GPIO_NUM_18
#define SB1 GPIO_NUM_19
#define SB2 GPIO_NUM_21

#define BARRERA_ABRIR GPIO_NUM_23
#define BARRERA_CERRAR GPIO_NUM_22
#define DURACION_PULSO_MS 600
#define ALARMA GPIO_NUM_2

// Variables compartidas
volatile bool solicitar_apertura = false;
volatile bool solicitar_cierre = false;
volatile bool barrera_en_movimiento = false;
volatile bool barrera_abierta = true;
volatile bool alarma_activada = false;

volatile bool sa1 = false, sa2 = false, sb1 = false, sb2 = false;

static button_t btn_sa1, btn_sa2, btn_sb1, btn_sb2;

// Estados del sistema
typedef enum {
    ESTADO_VALIDACION_FORZADA,
    ESTADO_PASO_PEATONAL,
    ESTADO_PROCESANDO_A_B,
    ESTADO_PROCESANDO_B_A,
    ESTADO_OPTIMIZACION_A_B,
    ESTADO_OPTIMIZACION_B_A
} estado_t;

static estado_t estado_actual = ESTADO_PASO_PEATONAL;
static TimerHandle_t timer_optimizacion;
static TimerHandle_t timer_seguridad;

void controlar_alarma(bool activar) {
    alarma_activada = activar;
    gpio_set_level(ALARMA, activar ? 1 : 0);
    ESP_LOGI(TAG, "Alarma %s", activar ? "ACTIVADA" : "DESACTIVADA");
}

void enviar_pulso_barrera(bool abrir) {
    gpio_num_t pin = abrir ? BARRERA_ABRIR : BARRERA_CERRAR;
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(DURACION_PULSO_MS));
    gpio_set_level(pin, 0);
}

void task_control_barreras(void *param) {
    while (1) {
        if (solicitar_cierre && !barrera_en_movimiento) {
            barrera_en_movimiento = true;
            solicitar_cierre = false;
            enviar_pulso_barrera(false);
            barrera_abierta = false;
            ESP_LOGI(TAG, "Barrera cerrada");
            barrera_en_movimiento = false;
        }

        if (solicitar_apertura && !barrera_en_movimiento) {
            barrera_en_movimiento = true;
            solicitar_apertura = false;
            enviar_pulso_barrera(true);
            barrera_abierta = true;
            ESP_LOGI(TAG, "Barrera abierta");
            barrera_en_movimiento = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void on_timer_optimizacion(TimerHandle_t xTimer) {
    controlar_alarma(false);
    solicitar_apertura = true;
    estado_actual = ESTADO_PASO_PEATONAL;
    ESP_LOGI(TAG, "Fin optimizacion - Regresando a paso peatonal");
}

void on_timer_seguridad(TimerHandle_t xTimer) {
    ESP_LOGW(TAG, "TIEMPO MÁXIMO DE CRUCE ALCANZADO - FORZANDO RESET");
    controlar_alarma(false);

    if (estado_actual != ESTADO_PASO_PEATONAL){
        solicitar_apertura = true;
    }
    estado_actual = ESTADO_PASO_PEATONAL;
}

void on_sensor_callback(button_t *btn, button_state_t state) {
    if (state != BUTTON_PRESSED_LONG) return;

    if (btn->gpio == SA1) sa1 = true;
    if (btn->gpio == SA2) sa2 = true;
    if (btn->gpio == SB1) sb1 = true;
    if (btn->gpio == SB2) sb2 = true;

    ESP_LOGW("SEN", "BTN[%02d] -> %02d",btn->gpio, state);

    if (timer_seguridad != NULL) {
        if (xTimerIsTimerActive(timer_seguridad)) {
            ESP_LOGI(TAG, "Security timer is running.");
            xTimerReset(timer_seguridad, 0);
        } else {
            ESP_LOGI(TAG, "Security timer is NOT running.");
            xTimerStart(timer_seguridad, 0);
        }
    } else {
        ESP_LOGW(TAG, "Security timer not initialized (NULL).");
    }

}

void task_procesar_sensores(void *param) {
    bool paso_a_b_detectado = false;
    bool paso_b_a_detectado = false;

    while (1) {
        switch (estado_actual) {
            case ESTADO_VALIDACION_FORZADA:
                if (sb2) {
                    ESP_LOGI(TAG, "Validación forzada resuelta: sentido A->B");
                    estado_actual = ESTADO_PROCESANDO_A_B;
                    ESP_LOGI(TAG, "Cambio de estado: PROCESANDO_A_B");
                    paso_a_b_detectado = true;
                    sb2 = false;
                } else if (sa1) {
                    ESP_LOGI(TAG, "Validación forzada resuelta: sentido B->A");
                    estado_actual = ESTADO_PROCESANDO_B_A;
                    ESP_LOGI(TAG, "Cambio de estado: PROCESANDO_B_A");
                    paso_b_a_detectado = true;
                    sa1 = false;
                }
                break;

            case ESTADO_PASO_PEATONAL:
                if (sa1) {
                    ESP_LOGI(TAG, "Detectado A->B");
                    controlar_alarma(true);
                    estado_actual = ESTADO_PROCESANDO_A_B;
                    ESP_LOGI(TAG, "Cambio de estado: PROCESANDO_A_B");
                    // xTimerStart(timer_seguridad, 0);
                    paso_a_b_detectado = false;
                    sa1 = false;
                } else if (sb2) {
                    ESP_LOGI(TAG, "Detectado B->A");
                    solicitar_cierre = true;
                    controlar_alarma(true);
                    estado_actual = ESTADO_PROCESANDO_B_A;
                    ESP_LOGI(TAG, "Cambio de estado: PROCESANDO_B_A");
                    // xTimerStart(timer_seguridad, 0);
                    paso_b_a_detectado = false;
                    sb2 = false;
                } else if (sa2 || sb1) {
                    ESP_LOGW(TAG, "Sensor intermedio activado tras timeout. Validación forzada");
                    solicitar_cierre = true;
                    controlar_alarma(true);
                    // xTimerStart(timer_seguridad, 0);
                    estado_actual = ESTADO_VALIDACION_FORZADA;
                    sa2 = sb1 = false;
                }
                break;

            case ESTADO_PROCESANDO_A_B:
                if (sa2) {
                    solicitar_cierre = true;
                    // xTimerReset(timer_seguridad, 0);
                    ESP_LOGI(TAG, "Timer de seguridad reiniciado");
                    ESP_LOGI(TAG, "A->B: confirmado en SA2 - cerrando barrera");
                    sa2 = false;
                }
                if (sb1) {
                    paso_a_b_detectado = true;
                    // xTimerReset(timer_seguridad, 0);
                    ESP_LOGI(TAG, "A->B: paso intermedio SB1");
                    sb1 = false;
                }
                if (sb2 && paso_a_b_detectado) {
                    controlar_alarma(false);
                    ESP_LOGI(TAG, "A->B: salida completa SB2");
                    // xTimerStop(timer_seguridad, 0);
                    ESP_LOGI(TAG, "Timer de seguridad detenido");
                    xTimerStart(timer_optimizacion, 0);
                    estado_actual = ESTADO_OPTIMIZACION_A_B;
                    ESP_LOGI(TAG, "Cambio de estado: OPTIMIZACION_A_B");
                    sb2 = false;
                }
                break;

            case ESTADO_PROCESANDO_B_A:
                if (sa2) {
                    solicitar_cierre = true;
                    paso_b_a_detectado = true;
                    ESP_LOGI(TAG, "B->A: paso intermedio SA2");
                    // xTimerReset(timer_seguridad, 0);
                    sa2 = false;
                }
                if (sa1 && paso_b_a_detectado) {
                    // xTimerStop(timer_seguridad, 0);
                    controlar_alarma(false);
                    ESP_LOGI(TAG, "B->A: salida completa SA1");
                    xTimerStart(timer_optimizacion, 0);
                    estado_actual = ESTADO_OPTIMIZACION_B_A;
                    ESP_LOGI(TAG, "Cambio de estado: OPTIMIZACION_B_A");
                    sa1 = false;
                }
                break;

            case ESTADO_OPTIMIZACION_A_B:
                if (sa1 || sa2) {
                    ESP_LOGI(TAG, "Nuevo vehiculo A->B durante optimizacion");
                    xTimerStop(timer_optimizacion, 0);
                    estado_actual = ESTADO_PROCESANDO_A_B;
                    ESP_LOGI(TAG, "Cambio de estado: PROCESANDO_A_B");
                    paso_a_b_detectado = false;
                    sa1 = sa2 = false;
                } else if (sb2) {
                    ESP_LOGI(TAG, "Vehiculo B->A llego - cambio de sentido");
                    xTimerStop(timer_optimizacion, 0);
                    estado_actual = ESTADO_PROCESANDO_B_A;
                    ESP_LOGI(TAG, "Cambio de estado: PROCESANDO_B_A");
                    solicitar_cierre = true;
                    paso_b_a_detectado = false;
                    sb2 = false;
                }
                break;

            case ESTADO_OPTIMIZACION_B_A:
                if (sb1 || sb2) {
                    ESP_LOGI(TAG, "Nuevo vehiculo B->A durante optimizacion");
                    xTimerStop(timer_optimizacion, 0);
                    estado_actual = ESTADO_PROCESANDO_B_A;
                    ESP_LOGI(TAG, "Cambio de estado: PROCESANDO_B_A");
                    paso_b_a_detectado = false;
                    sb1 = sb2 = false;
                } else if (sa1) {
                    ESP_LOGI(TAG, "Vehiculo A->B llego - cambio de sentido");
                    xTimerStop(timer_optimizacion, 0);
                    estado_actual = ESTADO_PROCESANDO_A_B;
                    ESP_LOGI(TAG, "Cambio de estado: PROCESANDO_A_B");
                    solicitar_cierre = true;
                    paso_a_b_detectado = false;
                    sa1 = false;
                }
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void init_gpio_con_button() {
    btn_sa1.gpio = SA1;
    btn_sa1.pressed_level = 0;
    btn_sa1.internal_pull = true;
    btn_sa1.autorepeat = false;
    btn_sa1.callback = on_sensor_callback;

    btn_sa2.gpio = SA2;
    btn_sa2.pressed_level = 0;
    btn_sa2.internal_pull = true;
    btn_sa2.autorepeat = false;
    btn_sa2.callback = on_sensor_callback;

    btn_sb1.gpio = SB1;
    btn_sb1.pressed_level = 0;
    btn_sb1.internal_pull = true;
    btn_sb1.autorepeat = false;
    btn_sb1.callback = on_sensor_callback;

    btn_sb2.gpio = SB2;
    btn_sb2.pressed_level = 0;
    btn_sb2.internal_pull = true;
    btn_sb2.autorepeat = false;
    btn_sb2.callback = on_sensor_callback;

    button_init(&btn_sa1);
    button_init(&btn_sa2);
    button_init(&btn_sb1);
    button_init(&btn_sb2);

    gpio_config_t out_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BARRERA_ABRIR) | (1ULL << BARRERA_CERRAR) |  (1ULL << ALARMA),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_conf);

    gpio_set_level(BARRERA_ABRIR, 0);
    gpio_set_level(BARRERA_CERRAR, 0);
    gpio_set_level(ALARMA, 0);
}

void task_logger_estado(void *param) {
    extern volatile bool sa1, sa2, sb1, sb2;
    extern volatile bool barrera_abierta, barrera_en_movimiento;
    while (1) {
        switch (estado_actual) {
            case ESTADO_PASO_PEATONAL:
                ESP_LOGI("LOGGER", "Estado: PASO_PEATONAL");
                break;
            case ESTADO_PROCESANDO_A_B:
                ESP_LOGI("LOGGER", "Estado: PROCESANDO_A_B");
                break;
            case ESTADO_PROCESANDO_B_A:
                ESP_LOGI("LOGGER", "Estado: PROCESANDO_B_A");
                break;

            case ESTADO_VALIDACION_FORZADA:
                ESP_LOGI("LOGGER", "Estado: VALIDACION_FORZADA");
                break;

            case ESTADO_OPTIMIZACION_A_B:
                ESP_LOGI("LOGGER", "Estado: OPTIMIZACION_A_B");
                break;
            case ESTADO_OPTIMIZACION_B_A:
                ESP_LOGI("LOGGER", "Estado: OPTIMIZACION_B_A");
                break;
        }
        ESP_LOGI("LOGGER", "Sensores: SA1=%d, SA2=%d, SB1=%d, SB2=%d", sa1, sa2, sb1, sb2);
        ESP_LOGI("LOGGER", "Barreras: %s | Movimiento: %s | Alarma: %s", 
                 barrera_abierta ? "ABIERTA" : "CERRADA", 
                 barrera_en_movimiento ? "EN MOVIMIENTO" : "DETENIDA",
                 alarma_activada ? "ACTIVA" : "APAGADA");
        vTaskDelay(pdMS_TO_TICKS(3000)); // Log cada 3 segundos
    }
}

void app_main() {
    
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Iniciando sistema de control de barreras...");
    init_gpio_con_button();

    timer_optimizacion = xTimerCreate("opt", pdMS_TO_TICKS(5000), pdFALSE, NULL, on_timer_optimizacion);
    timer_seguridad = xTimerCreate("seguridad", pdMS_TO_TICKS(20000), pdFALSE, NULL, on_timer_seguridad);

    xTaskCreate(task_control_barreras, "control_barreras", 2048, NULL, 10, NULL);
    xTaskCreate(task_procesar_sensores, "procesar_sensores", 4096, NULL, 9, NULL);
    xTaskCreate(task_logger_estado, "logger_estado", 2048, NULL, 5, NULL);
}
