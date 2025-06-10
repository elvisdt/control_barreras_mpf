#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_check.h"

#include <esp_idf_lib_helpers.h>
#include <button.h>

static const char *TAG = "CONTROL";

// ================================
// CONFIGURACIÓN DE PINES
// ================================
#define PIN_SENSOR_SA1         GPIO_NUM_4    // Sensor A1 (más alejado)
#define PIN_SENSOR_SA2         GPIO_NUM_18   // Sensor A2 (más cerca)
#define PIN_SENSOR_SB2         GPIO_NUM_19   // Sensor B2 (más cerca)
#define PIN_SENSOR_SB1         GPIO_NUM_21   // Sensor B1 (más alejado)

#define PIN_OUT_SEN_IR         GPIO_NUM_23   // Pulso para abrir barrera
#define PIN_BARRERA_CERRAR     GPIO_NUM_22   // Pulso para cerrar barrera
#define PIN_ADVERTENCIA_PEATONAL GPIO_NUM_25  // Luces + sirena advertencia peatonal

#define PIN_OUT_AUX01          GPIO_NUM_26     // OUT GND 01
#define PIN_OUT_AUX02          GPIO_NUM_2     // OUT GND 01

// Configuración de pulsos
#define DURACION_PULSO_MS      600          // Duración del pulso en ms

// ================================
// ESTRUCTURAS Y ENUMS
// ================================
typedef enum {
    GLOBAL_RESET = 0,     // Monitoreando ambos sentidos
    GLOBAL_WAIT,
    GLOBAL_S_AB,
    GLOBAL_S_BA,
} estado_global_t;

typedef enum {
    FASE_NA = 0, 
    FASE_A1, 
    FASE_A2,  
    FASE_B2,        
    FASE_B1,         
} fase_procesamiento_t;

typedef struct {
    bool sa1, sa2, sb1, sb2;
} sensores_t;

typedef struct {
    estado_global_t estado_global;
    fase_procesamiento_t fase_actual;
    bool barrier_state;
    bool leds_state;
} sistema_estado_t;

// ================================
// VARIABLES GLOBALES
// ================================
static sistema_estado_t g_sistema = {0};
static sensores_t g_sensores_actual = {0};

static esp_timer_handle_t timer_reset = NULL;
static esp_timer_handle_t timer_inactivity = NULL; 

// Botones/Sensores
static button_t btn_sa1, btn_sa2, btn_sb1, btn_sb2;

// Configuración de tiempos (en microsegundos)
#define TIEMPO_TIMEOUT_05     (5 * 1000000)  
#define TIEMPO_TIMEOUT_10     (10 * 1000000)  
#define TIEMPO_TIMEOUT_15     (15 * 1000000)  
#define TIEMPO_TIMEOUT_20     (20 * 1000000)  
#define TIEMPO_TIMEOUT_30     (30 * 1000000)  

// ================================
// DECLARACIONES DE FUNCIONES
// ================================
void abrir_barreras_y_resetear();
void procesar_cambio_sensor(gpio_num_t pin, bool estado);

void process_global_wait(gpio_num_t pin, bool estado);
void process_AB_and_get_state(gpio_num_t pin, bool estado);
void process_BA_and_get_state(gpio_num_t pin, bool estado);

void realice_action_fase_A_B(uint8_t fase);
void realice_action_fase_B_A(uint8_t fase);

// ================================
// FUNCIONES DE HARDWARE
// ================================
void init_gpio_outputs() {
    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << PIN_OUT_SEN_IR) | (1ULL << PIN_BARRERA_CERRAR) |
                        (1ULL << PIN_OUT_AUX01) | (1ULL << PIN_OUT_AUX02) |
                        (1ULL << PIN_ADVERTENCIA_PEATONAL),
        
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&output_config);

    gpio_set_level(PIN_OUT_SEN_IR, 0);
    gpio_set_level(PIN_BARRERA_CERRAR, 0);
    gpio_set_level(PIN_ADVERTENCIA_PEATONAL, 0);
    gpio_set_level(PIN_OUT_AUX01, 0);
    gpio_set_level(PIN_OUT_AUX02, 0);
    
    ESP_LOGI(TAG, "GPIO de salida configurados");
}

void set_barrier_state(bool abrir) {
    gpio_set_level(PIN_OUT_SEN_IR, abrir? 0:1);
    g_sistema.barrier_state = abrir;
    ESP_LOGE(TAG, ">>> %s BARRERAS", abrir ? "OPEN" : "CLOSE");
}

void set_leds_warning(bool activar) {
    gpio_set_level(PIN_ADVERTENCIA_PEATONAL, activar ? 1 : 0);
    gpio_set_level(PIN_OUT_AUX02, activar ? 1 : 0);
    g_sistema.leds_state = activar;
    ESP_LOGI(TAG, "--> LEDS: %s", activar ? "ON" : "OFF");
}


// ================================
// CALLBACKS DE TIMERS
// ================================
void callback_reset_timeout(void* arg) {
    ESP_LOGW(TAG, "TIMEOUT: No llego a sensor de confirmacion - RESET");
    abrir_barreras_y_resetear();
}


void callback_inactivity_timeout(void* arg) {
    ESP_LOGW(TAG, "RESET POR INACTIVIDAD: 30 seg sin actividad de sensores");
    abrir_barreras_y_resetear();
}

// Función para reiniciar timer de inactividad
void reiniciar_timer_inactividad() {
    if (timer_inactivity) {
        esp_timer_stop(timer_inactivity);
        esp_err_t err = esp_timer_start_once(timer_inactivity, TIEMPO_TIMEOUT_30);
        ESP_LOGI(TAG, "init inactivit timeout: %d", err);
    }
}

void reiniciar_timer_reset(uint64_t timeout_us) {
    if (timer_reset) {
        esp_timer_stop(timer_reset);
        esp_err_t err = esp_timer_start_once(timer_reset, timeout_us);
        ESP_LOGI(TAG, "init reset timeout: %d", err);
    }
}


// ================================
// CALLBACK PRINCIPAL DE SENSORES
// ================================
static void on_sensor_callback(button_t *btn, button_state_t state) {
    

    if (state != BUTTON_PRESSED && state != BUTTON_RELEASED) {
        return;
    }
    
    bool sensor_activo = (state == BUTTON_PRESSED);
    
    const char* nombre_sensor = "DESCONOCIDO";
    switch (btn->gpio) {
        case PIN_SENSOR_SA1: nombre_sensor = "SA1"; break;
        case PIN_SENSOR_SA2: nombre_sensor = "SA2"; break;
        case PIN_SENSOR_SB1: nombre_sensor = "SB1"; break;
        case PIN_SENSOR_SB2: nombre_sensor = "SB2"; break;
        default: break;
    }
    
    ESP_LOGW(TAG, "Sensor %s: %s", nombre_sensor, sensor_activo ? "ACTIVADO" : "DESACTIVADO");
    
    procesar_cambio_sensor(btn->gpio, sensor_activo);
}

// ================================
// FUNCIONES DE CONTROL
// ================================
void abrir_barreras_y_resetear() {
    // Detener todos los timers
    if (timer_inactivity) esp_timer_stop(timer_inactivity);
    if (timer_reset) esp_timer_stop(timer_reset);

    // Abrir barreras y apagar luces
    set_barrier_state(true);
    set_leds_warning(false);
    
    // Reset completo del sistema
    g_sistema.estado_global = GLOBAL_WAIT;
    g_sistema.fase_actual = FASE_NA;

    ESP_LOGI(TAG, "===== SISTEMA RESETEADO - PASO PEATONAL =====");
}


// ================================
// LÓGICA PRINCIPAL DEL SISTEMA
// ================================
void procesar_cambio_sensor(gpio_num_t pin, bool estado) {

    // REINICIAR TIMER GLOBAL SIEMPRE que haya actividad
    reiniciar_timer_inactividad();

    // Actualizar estado del sensor
    switch (pin) {
        case PIN_SENSOR_SA1: g_sensores_actual.sa1 = estado; break;
        case PIN_SENSOR_SA2: g_sensores_actual.sa2 = estado; break;
        case PIN_SENSOR_SB1: g_sensores_actual.sb1 = estado; break;
        case PIN_SENSOR_SB2: g_sensores_actual.sb2 = estado; break;
        default: return;
    }
    
    ESP_LOGI(TAG, "Event PRO: SA1=%d SA2=%d SB1=%d SB2=%d", 
             g_sensores_actual.sa1, g_sensores_actual.sa2, 
             g_sensores_actual.sb1, g_sensores_actual.sb2);
    
    // Máquina de estados principal
    switch (g_sistema.estado_global) {
        case GLOBAL_WAIT:
            process_global_wait(pin, estado);
            break;
            
        case GLOBAL_S_AB:
            process_AB_and_get_state(pin, estado);

            break;
            
        case GLOBAL_S_BA:
            process_BA_and_get_state(pin, estado);
            //realice_action_fase_B_A(g_sistema.fase_actual);
            break;
        default:
            abrir_barreras_y_resetear(); 
            break;
    }

    // realice action
    if (g_sistema.estado_global == GLOBAL_S_AB){
        realice_action_fase_A_B(g_sistema.fase_actual);
    }else if (g_sistema.estado_global == GLOBAL_S_BA){
        realice_action_fase_B_A(g_sistema.fase_actual);
    }
    
    
}


void process_global_wait(gpio_num_t pin, bool estado) {
    ESP_LOGW(TAG, "GB WAIT >> EVENT:%d ->pin: %d",estado, pin);

    if (estado && pin == PIN_SENSOR_SA1) {
        ESP_LOGI(TAG, "INIT DIR A->B: ON leds");
        reiniciar_timer_reset(TIEMPO_TIMEOUT_10);
        set_leds_warning(true);    // on leds 
        g_sistema.estado_global = GLOBAL_S_AB;
        g_sistema.fase_actual = FASE_A1;

    }else if (estado && pin== PIN_SENSOR_SB1){
        ESP_LOGI(TAG, "INIT DIR B->A: ON leds");
        reiniciar_timer_reset(TIEMPO_TIMEOUT_10);
        set_leds_warning(true);    // on leds 
        g_sistema.estado_global = GLOBAL_S_BA;
        g_sistema.fase_actual = FASE_B1;
    }else{
        ESP_LOGW(TAG, "UNKNOW EVENTS ON WAIT");
    }    
}

void process_AB_and_get_state(gpio_num_t pin, bool estado){
    g_sistema.estado_global = GLOBAL_S_AB;
    switch (g_sistema.fase_actual)
    {
    case FASE_A1:
        //--0---X---| |---0---0
        if (estado && pin == PIN_SENSOR_SA2) {
            ESP_LOGI(TAG, "INIT DIR A->B: ON leds");
            reiniciar_timer_reset(TIEMPO_TIMEOUT_10);            
            g_sistema.fase_actual = FASE_A2;
        //--X---0---| |---0---0
        }else if (estado && pin == PIN_SENSOR_SA1){
            reiniciar_timer_reset(TIEMPO_TIMEOUT_10);            
            g_sistema.fase_actual = FASE_A1;
            ESP_LOGW(TAG, "RETURN TO F A1");
        //--0---0---| |---0---0
        }else if (!estado && pin == PIN_SENSOR_SA1){
            reiniciar_timer_reset(TIEMPO_TIMEOUT_10);            
            g_sistema.fase_actual = FASE_A1;
            ESP_LOGW(TAG, "RETURN TIMEOUT REDUCE 10");
        }
        break;
    case FASE_A2:
        //--0---0---|x|---X---0
        if (estado && pin == PIN_SENSOR_SB2) {
            ESP_LOGI(TAG, "INIT DIR A->B: ON leds");
            reiniciar_timer_reset(TIEMPO_TIMEOUT_20);            
            g_sistema.fase_actual = FASE_B2;

        //--X---X---|x|---0---0
        }else if (estado && (pin == PIN_SENSOR_SA1 || pin == PIN_SENSOR_SA2)){
            reiniciar_timer_reset(TIEMPO_TIMEOUT_20);            
            g_sistema.fase_actual = FASE_A2;
            ESP_LOGW(TAG, "RETURN TO F A2");
        }
        break;

    case FASE_B2:
        //--0---0---|x|---0---X
        if (estado && pin == PIN_SENSOR_SB1) {
            ESP_LOGI(TAG, "INIT DIR A->B: ON leds");
            reiniciar_timer_reset(TIEMPO_TIMEOUT_10);            
            g_sistema.fase_actual = FASE_B1;
        
        //--X---X---|x|---0---0
        }else if (estado && (pin == PIN_SENSOR_SA1 || pin == PIN_SENSOR_SA2)){
            reiniciar_timer_reset(TIEMPO_TIMEOUT_20);            
            g_sistema.fase_actual = FASE_A2;
            ESP_LOGW(TAG, "RETURN TO F A2");
        }
        break;
    
    case FASE_B1:
         //--1---0---|x|---0---0
        if (estado && pin == PIN_SENSOR_SA1) {
            ESP_LOGI(TAG, "INIT DIR A->B: OFF ALL");
            reiniciar_timer_reset(TIEMPO_TIMEOUT_10);            
            g_sistema.fase_actual = FASE_A2;
        
        //--X---X---|x|---0---0
        }else if (estado &&  pin == PIN_SENSOR_SA2){
            reiniciar_timer_reset(TIEMPO_TIMEOUT_20);            
            g_sistema.fase_actual = FASE_B2;
            ESP_LOGW(TAG, "RETURN TO F B2");
        }
        break;

    default:
        break;
    }
    
    return;
}
void process_BA_and_get_state(gpio_num_t pin, bool estado){
    
    g_sistema.estado_global = GLOBAL_S_BA;
    switch (g_sistema.fase_actual)
    {
    case FASE_B1:
        //--0---0---| |---X---0
        if (estado && pin == PIN_SENSOR_SB2) {
            ESP_LOGI(TAG, "INIT DIR B->A: ON leds");
            reiniciar_timer_reset(TIEMPO_TIMEOUT_10);            
            g_sistema.fase_actual = FASE_B2;
        //--X---0---| |---0---0
        }else if (estado && pin == PIN_SENSOR_SA1){
            reiniciar_timer_reset(TIEMPO_TIMEOUT_10);            
            g_sistema.fase_actual = FASE_B1;
            ESP_LOGW(TAG, "RETURN TO F B1");
        //--0---0---| |---0---0
        }else if (!estado && pin == PIN_SENSOR_SA1){
            reiniciar_timer_reset(TIEMPO_TIMEOUT_10);            
            g_sistema.fase_actual = FASE_B1;
            ESP_LOGW(TAG, "RETURN TIMEOUT REDUCE 10");
        }
        break;
    case FASE_B2:
        //--0---X---|x|---0---0
        if (estado && pin == PIN_SENSOR_SA2) {
            ESP_LOGI(TAG, "INIT DIR B->A: ON BARR");
            reiniciar_timer_reset(TIEMPO_TIMEOUT_20);            
            g_sistema.fase_actual = FASE_A2;

        //--X---X---|x|---0---0
        }else if (estado && (pin == PIN_SENSOR_SB1 || pin == PIN_SENSOR_SB2)){
            reiniciar_timer_reset(TIEMPO_TIMEOUT_20);            
            g_sistema.fase_actual = FASE_B2;
            ESP_LOGW(TAG, "RETURN TO F B2");
        }
        break;

    case FASE_A2:
        //--X---0---|x|---0---0
        if (estado && pin == PIN_SENSOR_SA1) {
            ESP_LOGI(TAG, "DIR A->B: OFF all");
            reiniciar_timer_reset(TIEMPO_TIMEOUT_10);            
            g_sistema.fase_actual = FASE_A1;
        
        //--X---X---|x|---0---0
        }else if (estado && (pin == PIN_SENSOR_SB1 || pin == PIN_SENSOR_SB2)){
            reiniciar_timer_reset(TIEMPO_TIMEOUT_20);            
            g_sistema.fase_actual = FASE_A2;
            ESP_LOGW(TAG, "RETURN TO F A2");
        }
        break;
    
    case FASE_A1:
         //--1---0---|x|---0---0
        if (estado && pin == PIN_SENSOR_SB1) {
            ESP_LOGI(TAG, "INIT DIR B->A: ON leds");
            reiniciar_timer_reset(TIEMPO_TIMEOUT_10);            
            g_sistema.fase_actual = FASE_B2;
        
        //--X---X---|x|---0---0
        }else if (estado &&  pin == PIN_SENSOR_SB2){
            reiniciar_timer_reset(TIEMPO_TIMEOUT_20);            
            g_sistema.fase_actual = FASE_A2;
            ESP_LOGW(TAG, "RETURN TO F A2");
        }
        break;

    default:
        break;
    }
    
    return;
}


void realice_action_fase_A_B(uint8_t fase){

    ESP_LOGW(TAG, "GB A->B, fase [%d]", fase);

    switch (fase)
    {
    case FASE_A1:
        //--X---0---| |---0---0
        set_leds_warning(true);    // on leds 
        break;
    case FASE_A2:
        //--0---X---|x|---0---0
        set_leds_warning(true);    // on leds 
        set_barrier_state(false);  // close barr
        break;

    case FASE_B2:
        //--0---0---|x|---X---0
        set_leds_warning(false);    // of leds
        break;
    
    case FASE_B1:
        //--0---0---| |---0---1
        set_leds_warning(false);    // on leds 
        // set_barrier_state(false);  // close barr
        reiniciar_timer_reset(TIEMPO_TIMEOUT_05); // WAIT 5 SEG TO CLOSE
        break;

    default:
        break;
    }
    
}


void realice_action_fase_B_A(uint8_t fase){

    ESP_LOGW(TAG, "GB A->B, fase [%d]", fase);

    switch (fase)
    {
    case FASE_B1:
        //--X---0---| |---0---0
        set_leds_warning(true);    // on leds 
        break;
    case FASE_B2:
        //--0---X---|x|---0---0
        set_leds_warning(true);    // on leds 
        set_barrier_state(false);  // close barr
        break;

    case FASE_A2:
        //--0---0---|x|---X---0
        set_leds_warning(false);    // of leds
        break;
    
    case FASE_A1:
        //--0---0---| |---0---1
        set_leds_warning(false);    // on leds 
        // set_barrier_state(false);  // close barr
        reiniciar_timer_reset(TIEMPO_TIMEOUT_05); // WAIT 5 SEG TO CLOSE
        break;

    default:
        break;
    }
    
}
// ================================
// FUNCIONES DE UTILIDAD
// ================================
const char* obtener_nombre_estado_global(estado_global_t estado) {
    switch (estado) {
        case GLOBAL_RESET: return "GLOBAL_RESET";
        case GLOBAL_WAIT: return "GLOBAL_WAIT";
        case GLOBAL_S_AB: return "PROCESANDO_A->B";
        case GLOBAL_S_BA: return "PROCESANDO_B->A";
        default: return "DESCONOCIDO";
    }
}

const char* obtener_nombre_fase(fase_procesamiento_t fase) {
    switch (fase) {
        case FASE_NA: return "FASE_NA";
        case FASE_A1: return "FASE_A1";
        case FASE_A2: return "FASE_A2";
        case FASE_B2: return "FASE_B2";
        case FASE_B1: return "FASE_B1";
        default: return "DESCONOCIDO";
    }
}

void mostrar_estado_sistema() {
    ESP_LOGI(TAG, "=== ESTADO DEL SISTEMA ===");
    ESP_LOGI(TAG, "Global ST: %s", obtener_nombre_estado_global(g_sistema.estado_global));
    ESP_LOGI(TAG, "Fase Act : %s", obtener_nombre_fase(g_sistema.fase_actual));
    ESP_LOGI(TAG, "Barreras : %s", g_sistema.barrier_state ? "ABIERTAS" : "CERRADAS");
    ESP_LOGI(TAG, "Sirena   : %s", g_sistema.leds_state ? "ACTIVA" : "INACTIVA");

    ESP_LOGI(TAG, "Sens: SA1=%d SA2=%d SB1=%d SB2=%d", 
             g_sensores_actual.sa1, g_sensores_actual.sa2, 
             g_sensores_actual.sb1, g_sensores_actual.sb2);
    ESP_LOGI(TAG, "======================== \n");
}

// ================================
// INICIALIZACIÓN
// ================================
int init_sensores_como_botones() {
    // SA1
    btn_sa1.gpio = PIN_SENSOR_SA1;
    btn_sa1.pressed_level = 0;
    btn_sa1.internal_pull = true;
    btn_sa1.autorepeat = false;
    btn_sa1.callback = on_sensor_callback;

    // SA2
    btn_sa2.gpio = PIN_SENSOR_SA2;
    btn_sa2.pressed_level = 0;
    btn_sa2.internal_pull = true;
    btn_sa2.autorepeat = false;
    btn_sa2.callback = on_sensor_callback;

    // SB1
    btn_sb1.gpio = PIN_SENSOR_SB1;
    btn_sb1.pressed_level = 0;
    btn_sb1.internal_pull = true;
    btn_sb1.autorepeat = false;
    btn_sb1.callback = on_sensor_callback;

    // SB2
    btn_sb2.gpio = PIN_SENSOR_SB2;
    btn_sb2.pressed_level = 0;
    btn_sb2.internal_pull = true;
    btn_sb2.autorepeat = false;
    btn_sb2.callback = on_sensor_callback;

    ESP_ERROR_CHECK(button_init(&btn_sa1));
    ESP_ERROR_CHECK(button_init(&btn_sa2));
    ESP_ERROR_CHECK(button_init(&btn_sb1));
    ESP_ERROR_CHECK(button_init(&btn_sb2));

    ESP_LOGI(TAG, "Sensores configurados como botones");
    return 0;
}


void init_timers() {
    esp_err_t err;

    // Timer entrada timeout
    esp_timer_create_args_t timer_reset_args = {
        .callback = &callback_reset_timeout,
        .name = "cb_reset",
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,  // Importante
        .skip_unhandled_events = false      // Importante
    };
    err = esp_timer_create(&timer_reset_args, &timer_reset);
    ESP_LOGI(TAG, "create reset timer: %s", esp_err_to_name(err));

    // Timer cierre máximo
    esp_timer_create_args_t timer_inactivity_args = {
        .callback = &callback_inactivity_timeout,
        .name = "cb_inactivity",
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = false
    };
    err = esp_timer_create(&timer_inactivity_args, &timer_inactivity);
    ESP_LOGI(TAG, "create inactivity timer: %s", esp_err_to_name(err));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al crear timer inactividad: %s", esp_err_to_name(err));
    }
}

void task_monitoreo_sistema(void *pvParameters) {
    int contador = 0;
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(15000)); // Cada 15 segundos
        contador++;
        
        ESP_LOGI(TAG, "Heartbeat #%d - OK", contador);
        
        if (contador % 4 == 0) { // Cada minuto
            mostrar_estado_sistema();
        }
    }
}

void app_main() {
    ESP_LOGI(TAG, "Iniciando Sistema Vehicular Bidireccional");
    
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Inicializar componentes
    init_gpio_outputs();
    init_timers();
    init_sensores_como_botones();
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    // Estado inicial
    abrir_barreras_y_resetear();
    reiniciar_timer_reset(TIEMPO_TIMEOUT_10);

    // Crear task de monitoreo
    xTaskCreate(task_monitoreo_sistema, "monitoreo", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Sistema bidireccional iniciado correctamente");
    mostrar_estado_sistema();
    
}