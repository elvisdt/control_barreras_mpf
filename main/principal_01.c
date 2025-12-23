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

#define BARRERA_ABRIR       GPIO_NUM_23
#define BARRERA_CERRAR      GPIO_NUM_22
#define ALARMA              GPIO_NUM_25
#define PIN_OUT_AUX01       GPIO_NUM_2


#define DURACION_PULSO_MS   700

#define TIME_CONFIRM_END    5000    // 5seg

#define TIME_ALERT_20_SEC   20000    // 20 seg
#define TIME_ALERT_30_SEC   25000    // 20 seg


#define MAX_BTN_HISTORY 2
int last_buttons[MAX_BTN_HISTORY] = {-1, -1};  // Inicializar con -1 para indicar que no hay botones aún


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
    ESTADO_PASO_PEATONAL,
    ESTADO_PROCESANDO_A_B,
    ESTADO_PROCESANDO_B_A,
    ESTADO_OPTIMIZACION_A_B,
    ESTADO_OPTIMIZACION_B_A,
    ESTADO_VALIDACION_FORZADA,
    ESTADO_CONFIRMACION
} estado_t;

static estado_t estado_actual = ESTADO_PASO_PEATONAL;
static TimerHandle_t timer_confirmacion;
static TimerHandle_t timer_seguridad;


void update_button_history(int gpio) {
    last_buttons[0] = last_buttons[1];  // El último se convierte en el penúltimo
    last_buttons[1] = gpio;             // El nuevo botón se guarda como el más reciente
}

// TIMER CONFIG

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"

void startOrResetTimerMs(TimerHandle_t timer, uint32_t tiempo_ms, const char *tag) {
    if (timer != NULL) {
        TickType_t new_period = pdMS_TO_TICKS(tiempo_ms);
        if (xTimerIsTimerActive(timer)) {
            ESP_LOGI(tag, "Timer is running. Changing to %lu ms and resetting...", tiempo_ms);
        } else {
            ESP_LOGI(tag, "Timer is NOT running. Starting with %lu ms...", tiempo_ms);
        }
        if (xTimerChangePeriod(timer, new_period, 0) != pdPASS) {
            ESP_LOGE(tag, "Failed to change/start timer.");
        }
    } else {
        ESP_LOGW(tag, "Timer handle is NULL. Cannot change/start timer.");
    }
}

void startOrResetTimer(TimerHandle_t timer, const char *tag) {
    if (timer != NULL) {
        if (xTimerIsTimerActive(timer)) {
            ESP_LOGI(tag, "Timer is running. Resetting...");
            xTimerReset(timer, 0);
        } else {
            ESP_LOGI(tag, "Timer is NOT running. Starting...");
            xTimerStart(timer, 0);
        }
    } else {
        ESP_LOGW(tag, "Timer handle is NULL. Cannot start/reset.");
    }
}



void stopTimerIfRunning(TimerHandle_t timer, const char *tag) {
    if (timer != NULL) {
        if (xTimerIsTimerActive(timer)) {
            ESP_LOGI(tag, "Stopping active timer...");
            xTimerStop(timer, 0);
        } else {
            ESP_LOGI(tag, "Timer is already stopped.");
        }
    } else {
        ESP_LOGW(tag, "Timer handle is NULL. Cannot stop.");
    }
}

//-----------------------------------------------------//

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
            ESP_LOGE(TAG, "Barrera cerrada");
            barrera_en_movimiento = false;
        }

        if (solicitar_apertura && !barrera_en_movimiento) {
            barrera_en_movimiento = true;
            solicitar_apertura = false;
            enviar_pulso_barrera(true);
            barrera_abierta = true;
            ESP_LOGE(TAG, "Barrera abierta");
            controlar_alarma(false);
            barrera_en_movimiento = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void on_timer_confirmacion(TimerHandle_t xTimer) {
    if(estado_actual == ESTADO_CONFIRMACION){
        controlar_alarma(false);
        solicitar_apertura = true;
        estado_actual = ESTADO_PASO_PEATONAL;
        ESP_LOGW(TAG, "Fin confirmacion - return to peatonal");
    }else{
        ESP_LOGW(TAG, "Fin confirmacion - not acction");
    }
}

void on_timer_seguridad(TimerHandle_t xTimer) {
    ESP_LOGE(TAG, "TIEMPO SECURITY END - FORCE OPEN");
    controlar_alarma(false);

    // if (estado_actual != ESTADO_PASO_PEATONAL){    
    //     solicitar_apertura = true;
    // }else{
    //     ESP_LOGW(TAG, "TIEMPO SECURITY END - NOT FORSE OPEN");
    // }
    solicitar_apertura = true;
    estado_actual = ESTADO_PASO_PEATONAL;
}

void on_sensor_callback(button_t *btn, button_state_t state) {

    //ESP_LOGW("SEN", "BTN[%02d] -> %02d ", btn->gpio, state);

    if (state != BUTTON_PRESSED) return;
    
    if (btn->gpio == SA1) sa1 = true;
    if (btn->gpio == SA2) sa2 = true;
    if (btn->gpio == SB1) sb1 = true;
    if (btn->gpio == SB2) sb2 = true;

    update_button_history(btn->gpio);

    ESP_LOGW("SEN", "BTN[%02d] -> %02d | Last: %d, Prev: %d",
             btn->gpio, state, last_buttons[1], last_buttons[0]);
}

void task_procesar_sensores(void *param) {
    // bool paso_a_b_detectado = false;
    // bool paso_b_a_detectado = false;
    estado_t last_state = estado_actual;

    while (1) {
        if(last_state != estado_actual){
            last_state = estado_actual;
            ESP_LOGI(TAG,"NEW STATE: %d",estado_actual);

            // forzar apertura en estado peatonal
            // if (estado_actual == ESTADO_PASO_PEATONAL){
            //     controlar_alarma(false);
            //     solicitar_apertura = true;
            //     stopTimerIfRunning(timer_seguridad, "T-SEG");
            // }else{
            //     // startOrResetTimer(timer_seguridad, "T-SEG");
            //     // startOrResetTimerMs(timer_seguridad,TIME_ALERT_20_SEC,"T-SEG");
            // }
        }
        
        switch (estado_actual) {
            case ESTADO_PASO_PEATONAL:{
                if (sa1) {
                    ESP_LOGI(TAG, "Detectado A->B");
                    estado_actual = ESTADO_PROCESANDO_A_B;

                    ESP_LOGI(TAG, "Cambio de estado: PROCESANDO_A_B");
                    controlar_alarma(true);
                    sa1 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_20_SEC,"T-SEG");

                } else if (sb2) {
                    ESP_LOGI(TAG, "Detectado B->A");
                    estado_actual = ESTADO_PROCESANDO_B_A;

                    ESP_LOGI(TAG, "Cambio de estado: PROCESANDO_B_A");
                    solicitar_cierre = true;
                    controlar_alarma(true);
                    sb2 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_20_SEC,"T-SEG");

                } else if (sa2 || sb1) {
                    ESP_LOGW(TAG, "Sensor intermedio activado. Validación forzada");
                    estado_actual = ESTADO_VALIDACION_FORZADA;

                    solicitar_cierre = true;
                    controlar_alarma(true);
                    sa2 = sb1 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_30_SEC,"T-SEG");
                }
                }break;

            case ESTADO_PROCESANDO_A_B:{
                if (sa2) {
                    ESP_LOGI(TAG, "A->B: confirmado en SA2 - cerrando barrera");
                    estado_actual = ESTADO_OPTIMIZACION_A_B;

                    solicitar_cierre = true;
                    sa2 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_30_SEC,"T-SEG");
                }
                else if (sa1) {
                    ESP_LOGI(TAG, "A->B: paso inicial SA1");
                    sa1 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_20_SEC,"T-SEG");
                }
                else if (sb1 || sb2) {
                    ESP_LOGW(TAG, "Sensor intermedio activado. Validación forzada");
                    estado_actual = ESTADO_VALIDACION_FORZADA;

                    solicitar_cierre = true;
                    controlar_alarma(true);
                    sb1 = sb2 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_30_SEC,"T-SEG");
                }
                }break;

            case ESTADO_PROCESANDO_B_A:{
                if (sb1) {
                    ESP_LOGI(TAG, "B->A: confirmado en SB2 - cerrando barrera");
                    estado_actual = ESTADO_OPTIMIZACION_B_A;

                    solicitar_cierre = true;
                    sb1 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_30_SEC,"T-SEG");
                }
                else if (sb2) {
                    ESP_LOGI(TAG, "B->A: paso inicial SB1");
                    sb2 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_20_SEC,"T-SEG");
                }
                else if (sa1 || sa2) {
                    ESP_LOGW(TAG, "Sensor intermedio activado. Validación forzada");
                    estado_actual = ESTADO_VALIDACION_FORZADA;

                    solicitar_cierre = true;
                    controlar_alarma(true);
                    sa1 = sa2 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_30_SEC,"T-SEG");
                }
                
                }break;

            case ESTADO_OPTIMIZACION_A_B:{
                if (sb2) {
                    ESP_LOGI(TAG, "Confirmacion A->B finalizada");
                    estado_actual = ESTADO_CONFIRMACION;
                    ESP_LOGI(TAG, "Cambio de estado: ESTADO_CONFIRMACION");
                    controlar_alarma(false);
                    sb2 = false;
                    stopTimerIfRunning(timer_seguridad, "T-SEG");
                    startOrResetTimer(timer_confirmacion, "T-OUT");

                } else if (sb1) {
                    ESP_LOGI(TAG, "Vehiculo A->B en camino");
                    // solicitar_cierre = true;
                    sb1 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_20_SEC,"T-SEG");

                }else if (sa1 || sa2){
                    ESP_LOGW(TAG, "Sensores A activado. Validación forzada");
                    estado_actual = ESTADO_VALIDACION_FORZADA;

                    solicitar_cierre = true;
                    controlar_alarma(true);
                    sa1 = sa2 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_20_SEC,"T-SEG");
                }
                
                }break;

            case ESTADO_OPTIMIZACION_B_A:{
                if (sa1) {
                    ESP_LOGI(TAG, "Confirmacion A->B finalizada");
                    estado_actual = ESTADO_CONFIRMACION;
                    ESP_LOGI(TAG, "Cambio de estado: ESTADO_CONFIRMACION");
                    controlar_alarma(false);
                    sa1 = false;
                    stopTimerIfRunning(timer_seguridad, "T-SEG");
                    startOrResetTimer(timer_confirmacion, "T-OUT");
                } else if (sa2) {
                    ESP_LOGI(TAG, "Vehiculo A->B en camino");
                    // solicitar_cierre = true;
                    sa2 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_20_SEC,"T-SEG");

                }else if (sb1 || sb2){
                    ESP_LOGW(TAG, "Sensores B activado. Validación forzada");
                    estado_actual = ESTADO_VALIDACION_FORZADA;

                    solicitar_cierre = true;
                    controlar_alarma(true);
                    sb1 = sb2 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_30_SEC,"T-SEG");
                }
                }break;
            case ESTADO_VALIDACION_FORZADA:{
                if (sa1 || sb2) {
                    ESP_LOGI(TAG, "Confirmacion estado forzado");
                    int prev_buton = last_buttons[0];
                    if(prev_buton == SA2 || prev_buton == SB1){
                        estado_actual = ESTADO_CONFIRMACION;
                        ESP_LOGI(TAG, "Cambio de estado: ESTADO_CONFIRMACION");
                        controlar_alarma(false);
                        stopTimerIfRunning(timer_seguridad, "T-SEG");
                        startOrResetTimer(timer_confirmacion, "T-OUT");
                    }else{
                        controlar_alarma(true);
                        solicitar_cierre = true;
                        startOrResetTimerMs(timer_seguridad,TIME_ALERT_20_SEC,"T-SEG");
                    }
                    sa1 = sb2 = false;

                } else if (sa2 || sb1) {
                    ESP_LOGI(TAG, "estado forzado");
                    controlar_alarma(true);
                    solicitar_cierre = true;
                    sa2 = sb1 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_30_SEC,"T-SEG");
                }
                }break;
            case ESTADO_CONFIRMACION:{
                if (sa1 || sa2) {
                    ESP_LOGI(TAG, "active A sens return OP");
                    estado_actual = ESTADO_OPTIMIZACION_A_B;
                    ESP_LOGI(TAG, "Cambio de estado: ESTADO_OPTIMIZACION_A_B");
                    controlar_alarma(true);
                    solicitar_cierre = true;
                    sa1 = sa2 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_20_SEC,"T-SEG");
                } else if (sb1 || sb2) {
                    ESP_LOGI(TAG, "active B sens return OP");
                    estado_actual = ESTADO_OPTIMIZACION_B_A;
                    ESP_LOGI(TAG, "Cambio de estado: ESTADO_OPTIMIZACION_B_A");
                    controlar_alarma(true);
                    solicitar_cierre = true;
                    sb1 = sb2 = false;
                    startOrResetTimerMs(timer_seguridad,TIME_ALERT_20_SEC,"T-SEG");
                }
                }break;
            default:
                estado_actual = ESTADO_PASO_PEATONAL;
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
    controlar_alarma(false);
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
            case ESTADO_CONFIRMACION:
                ESP_LOGI("LOGGER", "Estado: CONFIRMACION");
                break;
            default:
                ESP_LOGI("LOGGER", "Estado: DESCONOCIDO");
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
    
    // vTaskDelay(pdMS_TO_TICKS(10000));
    // solicitar_apertura = true;
    
    init_gpio_con_button();
    timer_confirmacion = xTimerCreate("confir", pdMS_TO_TICKS(TIME_CONFIRM_END), pdFALSE, NULL, on_timer_confirmacion);
    timer_seguridad = xTimerCreate("seguridad", pdMS_TO_TICKS(TIME_ALERT_20_SEC), pdFALSE, NULL, on_timer_seguridad);

    xTaskCreate(task_control_barreras, "control_barreras", 2048*2, NULL, 10, NULL);
    xTaskCreate(task_procesar_sensores, "procesar_sensores", 4096*2, NULL, 9, NULL);
    xTaskCreate(task_logger_estado, "logger_estado", 2048, NULL, 5, NULL);

    startOrResetTimer(timer_seguridad,"INIT-TIMER");
}
