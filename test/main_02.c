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
#define PIN_SENSOR_SA1              GPIO_NUM_4    // Sensor A1 (más alejado)
#define PIN_SENSOR_SA2              GPIO_NUM_18   // Sensor A2 (más cerca)
#define PIN_SENSOR_SB1              GPIO_NUM_19   // Sensor B1 (más cerca)
#define PIN_SENSOR_SB2              GPIO_NUM_21   // Sensor B2 (más alejado)

#define PIN_BARRERA_ABRIR           GPIO_NUM_23   // Pulso para abrir barrera
#define PIN_BARRERA_CERRAR          GPIO_NUM_22   // Pulso para cerrar barrera
#define PIN_ADVERTENCIA_PEATONAL    GPIO_NUM_25  // Luces + sirena advertencia peatonal

#define PIN_OUT_AUX01               GPIO_NUM_26     // OUT GND 01
#define PIN_OUT_AUX02               GPIO_NUM_2     // OUT GND 01

// Configuración de pulsos
#define DURACION_PULSO_MS      600          // Duración del pulso en ms

// ================================
// ESTRUCTURAS Y ENUMS
// ================================
typedef enum {
    GLOBAL_PASO_PEATONAL = 0,     // Monitoreando ambos sentidos
    GLOBAL_PROCESANDO_A_B,        // Procesando solo A->B (ignora B->A)
    GLOBAL_PROCESANDO_B_A,        // Procesando solo B->A (ignora A->B)
    GLOBAL_OPTIMIZACION_A_B,      // A->B completó, esperando 5seg
    GLOBAL_OPTIMIZACION_B_A       // B->A completó, esperando 5seg
} estado_global_t;

typedef enum {
    FASE_DETECTANDO_ENTRADA = 0,  // Esperando primer sensor (SA1 o SB2)
    FASE_ESPERANDO_CONFIRMACION,  // Esperando segundo sensor (SA2 o SB1)
    FASE_VEHICULO_PASANDO,        // Barreras cerradas, vehículo en cruce
    FASE_ESPERANDO_SALIDA         // Esperando que termine de salir
} fase_procesamiento_t;

typedef struct {
    bool sa1, sa2, sb1, sb2;
} sensores_t;

typedef struct {
    estado_global_t estado_global;
    fase_procesamiento_t fase_actual;
    bool barreras_abiertas;
    bool advertencia_peatonal_activa;
    bool vehiculo_en_transito;
    int cola_a_hacia_b;              // Contador de vehículos esperando A->B
    int cola_b_hacia_a;              // Contador de vehículos esperando B->A
    uint64_t timestamp_entrada;      // Tiempo cuando se detectó entrada
    uint64_t timestamp_salida;       // Tiempo cuando se detectó salida
} sistema_estado_t;

// ================================
// VARIABLES GLOBALES
// ================================
static sistema_estado_t g_sistema = {0};
static sensores_t g_sensores_actual = {0};

static esp_timer_handle_t timer_entrada_timeout = NULL;
static esp_timer_handle_t timer_cierre_maximo = NULL;
static esp_timer_handle_t timer_optimizacion = NULL;
static esp_timer_handle_t timer_inactividad_global = NULL;

// Botones/Sensores
static button_t btn_sa1, btn_sa2, btn_sb1, btn_sb2;

// Configuración de tiempos (en microsegundos)
#define TIEMPO_ENTRADA_TIMEOUT      (15 * 1000000)   // 5 seg: SA1→SA2 o SB2→SB1
#define TIEMPO_CIERRE_MAXIMO        (20 * 1000000)  // 30 seg: Máximo barreras cerradas
#define TIEMPO_OPTIMIZACION         (5 * 1000000)   // 5 seg: Espera post-salida
#define TIEMPO_MAX_INACTIVIDAD      (30 * 1000000)   // 10 seg: incactivedad

// ================================
// DECLARACIONES DE FUNCIONES
// ================================
void abrir_barreras_y_resetear(void);
void cerrar_barreras(void);
void controlar_advertencia_peatonal(bool activar);
void procesar_cambio_sensor(gpio_num_t pin, bool estado);
void mostrar_estado_sistema(void);
const char* obtener_nombre_estado_global(estado_global_t estado);
const char* obtener_nombre_fase(fase_procesamiento_t fase);
void procesar_paso_peatonal(gpio_num_t pin, bool estado) ;
void procesar_sentido_a_hacia_b(gpio_num_t pin, bool estado);
void procesar_sentido_b_hacia_a(gpio_num_t pin, bool estado);
void procesar_optimizacion_a_b(gpio_num_t pin, bool estado);
void procesar_optimizacion_b_a(gpio_num_t pin, bool estado);

// ================================
// FUNCIONES DE HARDWARE
// ================================
void init_gpio_outputs() {
    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << PIN_BARRERA_ABRIR) | (1ULL << PIN_BARRERA_CERRAR) |
                        (1ULL << PIN_OUT_AUX01) | (1ULL << PIN_OUT_AUX02) |
                        (1ULL << PIN_ADVERTENCIA_PEATONAL),
        
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&output_config);

    gpio_set_level(PIN_BARRERA_ABRIR, 0);
    gpio_set_level(PIN_BARRERA_CERRAR, 0);
    gpio_set_level(PIN_ADVERTENCIA_PEATONAL, 0);
    gpio_set_level(PIN_OUT_AUX01, 0);
    gpio_set_level(PIN_OUT_AUX02, 0);
    
    ESP_LOGI(TAG, "GPIO de salida configurados");
}

void enviar_pulso_barrera(bool abrir) {
    gpio_num_t pin = abrir ? PIN_BARRERA_ABRIR : PIN_BARRERA_CERRAR;
    
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(DURACION_PULSO_MS));
    gpio_set_level(pin, 0);
    
    g_sistema.barreras_abiertas = abrir;
    ESP_LOGE(TAG, ">>> %s BARRERAS", abrir ? "ABRIENDO" : "CERRANDO");
}

void controlar_advertencia_peatonal(bool activar) {
    gpio_set_level(PIN_ADVERTENCIA_PEATONAL, activar ? 1 : 0);
    g_sistema.advertencia_peatonal_activa = activar;
    ESP_LOGI(TAG, "Luces advertencia: %s", activar ? "ON" : "OFF");
}

// ================================
// CALLBACKS DE TIMERS
// ================================
void callback_entrada_timeout(void* arg) {
    ESP_LOGW(TAG, "TIMEOUT: No llego a sensor de confirmacion - RESET");
    abrir_barreras_y_resetear();
}

void callback_cierre_maximo(void* arg) {
    ESP_LOGW(TAG, "EMERGENCIA: Tiempo maximo cerrado excedido - APERTURA FORZADA");
    abrir_barreras_y_resetear();
}

void callback_optimizacion(void* arg) {
    ESP_LOGW(TAG, "OPTIMIZACION: No hay nuevos vehiculos - ABRIENDO");
    abrir_barreras_y_resetear();
}

void callback_inactividad_global(void* arg) {
    ESP_LOGW(TAG, "RESET POR INACTIVIDAD: 30 seg sin actividad de sensores");
    abrir_barreras_y_resetear();
}

// Función para reiniciar timer de inactividad
void reiniciar_timer_inactividad() {
    if (timer_inactividad_global) {
        esp_timer_stop(timer_inactividad_global);
        esp_err_t err = esp_timer_start_once(timer_inactividad_global, TIEMPO_MAX_INACTIVIDAD);
        ESP_LOGW(TAG, "init inactivit timeout: %d", err);
    }
}


// ================================
// CALLBACK PRINCIPAL DE SENSORES
// ================================
static void on_sensor_callback(button_t *btn, button_state_t state) {
    

    // if (state != BUTTON_PRESSED && state != BUTTON_RELEASED) {
    //     return;
    // }

    if (state != BUTTON_PRESSED) {
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
    if (timer_entrada_timeout) esp_timer_stop(timer_entrada_timeout);
    if (timer_cierre_maximo) esp_timer_stop(timer_cierre_maximo);
    if (timer_optimizacion) esp_timer_stop(timer_optimizacion);
    if (timer_inactividad_global) esp_timer_stop(timer_inactividad_global);
    
    // Abrir barreras y apagar luces
    enviar_pulso_barrera(true);
    controlar_advertencia_peatonal(false);
    
    // Reset completo del sistema
    g_sistema.estado_global = GLOBAL_PASO_PEATONAL;
    g_sistema.fase_actual = FASE_DETECTANDO_ENTRADA;
    g_sistema.barreras_abiertas = true;
    g_sistema.advertencia_peatonal_activa = false;
    g_sistema.vehiculo_en_transito = false;
    g_sistema.cola_a_hacia_b = 0;
    g_sistema.cola_b_hacia_a = 0;
    g_sistema.timestamp_entrada = 0;
    g_sistema.timestamp_salida = 0;
    
    ESP_LOGI(TAG, "===== SISTEMA RESETEADO - PASO PEATONAL =====");
}

void cerrar_barreras() {
    enviar_pulso_barrera(false);
    g_sistema.vehiculo_en_transito = true;
    g_sistema.fase_actual = FASE_VEHICULO_PASANDO;
    
    // Iniciar timer de seguridad máximo
    if (timer_cierre_maximo) {
        esp_timer_stop(timer_cierre_maximo);
        esp_timer_start_once(timer_cierre_maximo, TIEMPO_CIERRE_MAXIMO);
    }
}

void iniciar_procesamiento_sentido(estado_global_t nuevo_estado) {
    g_sistema.estado_global = nuevo_estado;
    g_sistema.fase_actual = FASE_ESPERANDO_CONFIRMACION;
    g_sistema.timestamp_entrada = esp_timer_get_time();
    
    controlar_advertencia_peatonal(true);
    
    // Timer para timeout entrada→confirmación
    if (timer_entrada_timeout) {
        esp_timer_stop(timer_entrada_timeout);
        esp_timer_start_once(timer_entrada_timeout, TIEMPO_ENTRADA_TIMEOUT);
    }
    
    const char* sentido = (nuevo_estado == GLOBAL_PROCESANDO_A_B) ? "A->B" : "B->A";
    ESP_LOGI(TAG, ">>> INICIANDO PROCESAMIENTO %s", sentido);
}

void iniciar_optimizacion(estado_global_t estado_optimizacion) {
    g_sistema.estado_global = estado_optimizacion;
    g_sistema.timestamp_salida = esp_timer_get_time();
    
    // Apagar luces inmediatamente cuando sale
    controlar_advertencia_peatonal(false);
    
    // Timer de optimización
    if (timer_optimizacion) {
        esp_timer_stop(timer_optimizacion);
        esp_timer_start_once(timer_optimizacion, TIEMPO_OPTIMIZACION);
    }
    
    const char* sentido = (estado_optimizacion == GLOBAL_OPTIMIZACION_A_B) ? "A->B" : "B->A";
    ESP_LOGI(TAG, ">>> OPTIMIZACION %s: Esperando 5seg para cerrar ciclo", sentido);
    // enviar_pulso_barrera(true);
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
        case GLOBAL_PASO_PEATONAL:
            procesar_paso_peatonal(pin, estado);
            break;
            
        case GLOBAL_PROCESANDO_A_B:
            procesar_sentido_a_hacia_b(pin, estado);
            break;
            
        case GLOBAL_PROCESANDO_B_A:
            procesar_sentido_b_hacia_a(pin, estado);
            break;
            
        case GLOBAL_OPTIMIZACION_A_B:
            procesar_optimizacion_a_b(pin, estado);
            break;
            
        case GLOBAL_OPTIMIZACION_B_A:
            procesar_optimizacion_b_a(pin, estado);
            break;
        default: break;
    }
}

void procesar_paso_peatonal(gpio_num_t pin, bool estado) {
    if (!estado) return; // Solo procesar activaciones
    
    // Detección de entrada por ambos sentidos
    if (pin == PIN_SENSOR_SA1) {
        ESP_LOGI(TAG, "DETECCION A->B: Vehiculo desde A");
        iniciar_procesamiento_sentido(GLOBAL_PROCESANDO_A_B);
        
    } else if (pin == PIN_SENSOR_SB2) {
        ESP_LOGI(TAG, "DETECCION B->A: Vehiculo desde B");
        iniciar_procesamiento_sentido(GLOBAL_PROCESANDO_B_A);
        cerrar_barreras();
    } else {
        ESP_LOGW(TAG, "PATRON EXTRANO: Sensor %d activo en paso peatonal", pin);
    }
}

void procesar_sentido_a_hacia_b(gpio_num_t pin, bool estado) {
    switch (g_sistema.fase_actual) {
        case FASE_ESPERANDO_CONFIRMACION:
            if (estado && pin == PIN_SENSOR_SA2) {
                ESP_LOGI(TAG, "CONFIRMACION A->B: Cerrando barreras");
                esp_timer_stop(timer_entrada_timeout); // Cancelar timeout
                cerrar_barreras();
            } else if (estado && pin == PIN_SENSOR_SA1) {
                ESP_LOGI(TAG, "COLA A->B: Vehiculo adicional detectado");
                g_sistema.cola_a_hacia_b++;
            }
            break;
            
        case FASE_VEHICULO_PASANDO:
            if (estado && pin == PIN_SENSOR_SB1) {
                ESP_LOGI(TAG, "A->B: Vehiculo llego a SB1");
                g_sistema.fase_actual = FASE_ESPERANDO_SALIDA;
            } else if (estado && (pin == PIN_SENSOR_SA1 || pin == PIN_SENSOR_SA2)) {
                ESP_LOGI(TAG, "COLA A->B: Vehiculo adicional en cola");
                g_sistema.cola_a_hacia_b++;
            }
            break;
            
        case FASE_ESPERANDO_SALIDA:
            if (estado && pin == PIN_SENSOR_SB2) {
                ESP_LOGI(TAG, "A->B: Vehiculo salio por SB2 - Iniciando optimizacion");
                iniciar_optimizacion(GLOBAL_OPTIMIZACION_A_B);
            } else if (estado && (pin == PIN_SENSOR_SA1 || pin == PIN_SENSOR_SA2)) {
                ESP_LOGI(TAG, "COLA A->B: Vehiculo adicional mientras sale");
                g_sistema.cola_a_hacia_b++;
            }
            break;
        default: break;
    }
}

void procesar_sentido_b_hacia_a(gpio_num_t pin, bool estado) {
    switch (g_sistema.fase_actual) {
        case FASE_ESPERANDO_CONFIRMACION:
            if (estado && pin == PIN_SENSOR_SB1) {
                ESP_LOGI(TAG, "CONFIRMACION B->A: Cerrando barreras");
                esp_timer_stop(timer_entrada_timeout); // Cancelar timeout
                cerrar_barreras();
            } else if (estado && pin == PIN_SENSOR_SB2) {
                ESP_LOGI(TAG, "COLA B->A: Vehiculo adicional detectado");
                g_sistema.cola_b_hacia_a++;
            }
            break;
            
        case FASE_VEHICULO_PASANDO:
            if (estado && pin == PIN_SENSOR_SA2) {
                ESP_LOGI(TAG, "B->A: Vehiculo llego a SA2");
                g_sistema.fase_actual = FASE_ESPERANDO_SALIDA;
            } else if (estado && (pin == PIN_SENSOR_SB1 || pin == PIN_SENSOR_SB2)) {
                ESP_LOGI(TAG, "COLA B->A: Vehiculo adicional en cola");
                g_sistema.cola_b_hacia_a++;
            }
            break;
            
        case FASE_ESPERANDO_SALIDA:
            if (estado && pin == PIN_SENSOR_SA1) {
                ESP_LOGI(TAG, "B->A: Vehiculo salio por SA1 - Iniciando optimizacion");
                iniciar_optimizacion(GLOBAL_OPTIMIZACION_B_A);
            } else if (estado && (pin == PIN_SENSOR_SB1 || pin == PIN_SENSOR_SB2)) {
                ESP_LOGI(TAG, "COLA B->A: Vehiculo adicional mientras sale");
                g_sistema.cola_b_hacia_a++;
            }
            break;
        default: break;
    }
}

void procesar_optimizacion_a_b(gpio_num_t pin, bool estado) {
    if (!estado) return; // Solo activaciones
    
    if (pin == PIN_SENSOR_SA1 || pin == PIN_SENSOR_SA2) {
        ESP_LOGI(TAG, "OPTIMIZACION A->B CANCELADA: Nuevo vehiculo A->B detectado");
        esp_timer_stop(timer_optimizacion);
        g_sistema.cola_a_hacia_b++;
        
        // Volver a procesamiento A->B
        g_sistema.estado_global = GLOBAL_PROCESANDO_A_B;
        g_sistema.fase_actual = FASE_ESPERANDO_CONFIRMACION;
        controlar_advertencia_peatonal(true);
        
        if (pin == PIN_SENSOR_SA1) {
            // Nuevo ciclo completo
            g_sistema.timestamp_entrada = esp_timer_get_time();
            if (timer_entrada_timeout) {
                esp_timer_start_once(timer_entrada_timeout, TIEMPO_ENTRADA_TIMEOUT);
            }
        } else {
            // SA2 directo, cerrar inmediatamente
            cerrar_barreras();
        }
        
    } else if (pin == PIN_SENSOR_SB2) {
        ESP_LOGI(TAG, "OPTIMIZACION A->B CANCELADA: Nuevo vehiculo B->A detectado");
        esp_timer_stop(timer_optimizacion);
        g_sistema.cola_b_hacia_a++;
        
        // Cambiar a procesamiento B->A
        iniciar_procesamiento_sentido(GLOBAL_PROCESANDO_B_A);
    }
}

void procesar_optimizacion_b_a(gpio_num_t pin, bool estado) {
    if (!estado) return; // Solo activaciones
    
    if (pin == PIN_SENSOR_SB1 || pin == PIN_SENSOR_SB2) {
        ESP_LOGI(TAG, "OPTIMIZACION B->A CANCELADA: Nuevo vehiculo B->A detectado");
        esp_timer_stop(timer_optimizacion);
        g_sistema.cola_b_hacia_a++;
        
        // Volver a procesamiento B->A
        g_sistema.estado_global = GLOBAL_PROCESANDO_B_A;
        g_sistema.fase_actual = FASE_ESPERANDO_CONFIRMACION;
        controlar_advertencia_peatonal(true);
        //cerrar_barreras();

        if (pin == PIN_SENSOR_SB2) {
            // Nuevo ciclo completo
            g_sistema.timestamp_entrada = esp_timer_get_time();
            if (timer_entrada_timeout) {
                esp_err_t err= esp_timer_start_once(timer_entrada_timeout, TIEMPO_ENTRADA_TIMEOUT);
                ESP_LOGW(TAG, "ESP LOG, INIT TIMER: %d",err);
            }
        } else {
            // SB1 directo, cerrar inmediatamente
            cerrar_barreras();
        }
        
    } else if (pin == PIN_SENSOR_SA1) {
        ESP_LOGI(TAG, "OPTIMIZACION B->A CANCELADA: Nuevo vehiculo A->B detectado");
        esp_timer_stop(timer_optimizacion);
        g_sistema.cola_a_hacia_b++;
        
        // Cambiar a procesamiento A->B
        iniciar_procesamiento_sentido(GLOBAL_PROCESANDO_A_B);
    }
}

// ================================
// FUNCIONES DE UTILIDAD
// ================================
const char* obtener_nombre_estado_global(estado_global_t estado) {
    switch (estado) {
        case GLOBAL_PASO_PEATONAL: return "PASO_PEATONAL";
        case GLOBAL_PROCESANDO_A_B: return "PROCESANDO_A->B";
        case GLOBAL_PROCESANDO_B_A: return "PROCESANDO_B->A";
        case GLOBAL_OPTIMIZACION_A_B: return "OPTIMIZACION_A->B";
        case GLOBAL_OPTIMIZACION_B_A: return "OPTIMIZACION_B->A";
        default: return "DESCONOCIDO";
    }
}

const char* obtener_nombre_fase(fase_procesamiento_t fase) {
    switch (fase) {
        case FASE_DETECTANDO_ENTRADA: return "DETECTANDO_ENTRADA";
        case FASE_ESPERANDO_CONFIRMACION: return "ESPERANDO_CONFIRMACION";
        case FASE_VEHICULO_PASANDO: return "VEHICULO_PASANDO";
        case FASE_ESPERANDO_SALIDA: return "ESPERANDO_SALIDA";
        default: return "DESCONOCIDO";
    }
}

void mostrar_estado_sistema() {
    ESP_LOGI(TAG, "=== ESTADO DEL SISTEMA ===");
    ESP_LOGI(TAG, "Global ST: %s", obtener_nombre_estado_global(g_sistema.estado_global));
    ESP_LOGI(TAG, "Fase Act : %s", obtener_nombre_fase(g_sistema.fase_actual));
    ESP_LOGI(TAG, "Barreras : %s", g_sistema.barreras_abiertas ? "ABIERTAS" : "CERRADAS");
    ESP_LOGI(TAG, "Sirena   : %s", g_sistema.advertencia_peatonal_activa ? "ACTIVA" : "INACTIVA");
    ESP_LOGI(TAG, "Cola A->B: %d", g_sistema.cola_a_hacia_b);
    ESP_LOGI(TAG, "Cola B->A: %d", g_sistema.cola_b_hacia_a);
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
    esp_timer_create_args_t timer_entrada_args = {
        .callback = &callback_entrada_timeout,
        .name = "entrada_timeout",
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,  // Importante
        .skip_unhandled_events = false      // Importante
    };
    err = esp_timer_create(&timer_entrada_args, &timer_entrada_timeout);
    ESP_LOGI(TAG, "create entrada timer: %s", esp_err_to_name(err));

    // Timer cierre máximo
    esp_timer_create_args_t timer_cierre_args = {
        .callback = &callback_cierre_maximo,
        .name = "cierre_maximo",
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = false
    };
    err = esp_timer_create(&timer_cierre_args, &timer_cierre_maximo);
    ESP_LOGI(TAG, "create cierre timer: %s", esp_err_to_name(err));

    // Timer optimización
    esp_timer_create_args_t timer_optimizacion_args = {
        .callback = &callback_optimizacion,
        .name = "optimizacion",
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = false
    };
    err = esp_timer_create(&timer_optimizacion_args, &timer_optimizacion);
    ESP_LOGI(TAG, "create optimizacion timer: %s", esp_err_to_name(err));

    // Timer inactividad global
    esp_timer_create_args_t timer_inactividad_args = {
        .callback = &callback_inactividad_global,
        .name = "inactividad",
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = false
    };
    err = esp_timer_create(&timer_inactividad_args, &timer_inactividad_global);
    ESP_LOGI(TAG, "create inactividad timer: %s", esp_err_to_name(err));

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
    
    vTaskDelay(pdMS_TO_TICKS(5000));
    // Estado inicial
    abrir_barreras_y_resetear();
    
    // Crear task de monitoreo
    xTaskCreate(task_monitoreo_sistema, "monitoreo", 1024*8, NULL, 5, NULL);
    ESP_LOGI(TAG, "Sistema bidireccional iniciado correctamente");
    mostrar_estado_sistema();    
}