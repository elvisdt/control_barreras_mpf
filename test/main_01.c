#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_check.h"

#include <esp_idf_lib_helpers.h>
#include <button.h>

static const char *TAG = "CONTROL_VEHICULAR";

// ================================
// CONFIGURACIÓN DE PINES
// ================================
#define PIN_SENSOR_SA1         GPIO_NUM_4    // Sensor A1 (más alejado)
#define PIN_SENSOR_SA2         GPIO_NUM_18   // Sensor A2 (más cerca)
#define PIN_SENSOR_SB1         GPIO_NUM_19   // Sensor B1 (más cerca)
#define PIN_SENSOR_SB2         GPIO_NUM_21   // Sensor B2 (más alejado)

#define PIN_BARRERA_ABRIR      GPIO_NUM_22   // Pulso para abrir barrera
#define PIN_BARRERA_CERRAR     GPIO_NUM_23   // Pulso para cerrar barrera
#define PIN_ADVERTENCIA_PEATONAL GPIO_NUM_2  // Luces + sirena advertencia peatonal

// Configuración de pulsos
#define DURACION_PULSO_MS      500          // Duración del pulso en ms

// ================================
// ESTRUCTURAS Y ENUMS
// ================================
typedef enum {
    SENTIDO_NINGUNO = 0,
    SENTIDO_A_HACIA_B,
    SENTIDO_B_HACIA_A
} sentido_t;

typedef enum {
    ESTADO_PASO_PEATONAL = 0,
    ESTADO_ALERTA_VEHICULO,
    ESTADO_PASO_VEHICULAR
} estado_sistema_t;

typedef struct {
    bool sa1, sa2, sb1, sb2;
} sensores_t;

typedef struct {
    bool barreras_abiertas;           // Estado lógico
    bool advertencia_peatonal_activa; // Luces + sirena
    sentido_t sentido_actual;
    bool vehiculo_en_transito;
    bool vehiculo_paso_completo;
    estado_sistema_t estado;
    uint64_t timestamp_inicio_advertencia;
    uint64_t timestamp_ultimo_sensor;
} sistema_estado_t;

// ================================
// VARIABLES GLOBALES
// ================================
static sistema_estado_t g_sistema = {0};
static sensores_t g_sensores_actual = {0};
static esp_timer_handle_t timer_cierre_barrera = NULL;
static esp_timer_handle_t timer_inactividad = NULL;

// Botones/Sensores
static button_t btn_sa1, btn_sa2, btn_sb1, btn_sb2;

// Configuración de tiempos (en microsegundos)
#define TIEMPO_RETARDO_SEGURIDAD    (5 * 1000000)  // 5 segundos
#define TIEMPO_CIERRE_DEFECTO       (30 * 1000000) // 30 segundos  
#define TIEMPO_CIERRE_OPTIMIZADO    (5 * 1000000)  // 5 segundos
#define TIEMPO_CIERRE_MULTIPLE      (10 * 1000000) // 10 segundos
#define TIEMPO_INACTIVIDAD_MAX      (30 * 1000000) // 30 segundos

// ================================
// DECLARACIONES DE FUNCIONES
// ================================
void abrir_barreras_y_resetear(void);
void enviar_pulso_barrera(bool abrir);
void controlar_advertencia_peatonal(bool activar);
void iniciar_timer_cierre(uint64_t tiempo_us);
void iniciar_timer_inactividad(void);
void mostrar_estado_sistema(void);
void procesar_cambio_sensor(gpio_num_t pin, bool estado);
void procesar_deteccion_aproximacion(sensores_t *sensores);
void procesar_confirmacion_paso(sensores_t *sensores);
void procesar_paso_completo(sensores_t *sensores);
void procesar_multiples_vehiculos(sensores_t *sensores);
void verificar_apertura_inmediata(sensores_t *sensores);
bool todos_sensores_inactivos(sensores_t *sensores);

// ================================
// FUNCIONES DE HARDWARE
// ================================
void init_gpio_outputs() {
    // Configurar solo las salidas (los sensores los maneja la librería)
    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << PIN_BARRERA_ABRIR) | (1ULL << PIN_BARRERA_CERRAR) |
                       (1ULL << PIN_ADVERTENCIA_PEATONAL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&output_config);

    // Estado inicial: todo apagado
    gpio_set_level(PIN_BARRERA_ABRIR, 0);
    gpio_set_level(PIN_BARRERA_CERRAR, 0);
    gpio_set_level(PIN_ADVERTENCIA_PEATONAL, 0);
    
    ESP_LOGI(TAG, "GPIO de salida configurados correctamente");
}

void enviar_pulso_barrera(bool abrir) {
    gpio_num_t pin = abrir ? PIN_BARRERA_ABRIR : PIN_BARRERA_CERRAR;
    
    // Enviar pulso
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(DURACION_PULSO_MS));
    gpio_set_level(pin, 0);
    
    // Actualizar estado lógico
    g_sistema.barreras_abiertas = abrir;
    
    ESP_LOGI(TAG, "Pulso enviado: %s barrera", abrir ? "ABRIR" : "CERRAR");
}

void controlar_advertencia_peatonal(bool activar) {
    gpio_set_level(PIN_ADVERTENCIA_PEATONAL, activar ? 1 : 0);
    g_sistema.advertencia_peatonal_activa = activar;
    ESP_LOGI(TAG, "Advertencia peatonal: %s", activar ? "ACTIVADA" : "DESACTIVADA");
}

// ================================
// CALLBACKS DE TIMERS
// ================================
void callback_cierre_barrera(void* arg) {
    ESP_LOGI(TAG, "Timer cierre barrera expirado - Abriendo barreras");
    abrir_barreras_y_resetear();
}

void callback_inactividad(void* arg) {
    ESP_LOGW(TAG, "EMERGENCIA! Timeout por inactividad - Apertura forzada");
    abrir_barreras_y_resetear();
}

// ================================
// CALLBACK PRINCIPAL DE SENSORES
// ================================
static void on_sensor_callback(button_t *btn, button_state_t state) {
    // Solo procesar PRESSED y RELEASED
    if (state != BUTTON_PRESSED && state != BUTTON_RELEASED) {
        return;
    }
    
    bool sensor_activo = (state == BUTTON_PRESSED);
    
    // Log del cambio
    const char* nombre_sensor = "DESCONOCIDO";
    switch (btn->gpio) {
        case PIN_SENSOR_SA1: nombre_sensor = "SA1"; break;
        case PIN_SENSOR_SA2: nombre_sensor = "SA2"; break;
        case PIN_SENSOR_SB1: nombre_sensor = "SB1"; break;
        case PIN_SENSOR_SB2: nombre_sensor = "SB2"; break;
        default:    
            break;
    }
    
    ESP_LOGI(TAG, "Sensor %s: %s", nombre_sensor, sensor_activo ? "ACTIVADO" : "DESACTIVADO");
    
    // Procesar el cambio
    procesar_cambio_sensor(btn->gpio, sensor_activo);
}

// ================================
// LÓGICA PRINCIPAL DEL SISTEMA
// ================================
void procesar_cambio_sensor(gpio_num_t pin, bool estado) {
    // Actualizar estado del sensor
    switch (pin) {
        case PIN_SENSOR_SA1:
            g_sensores_actual.sa1 = estado;
            break;
        case PIN_SENSOR_SA2:
            g_sensores_actual.sa2 = estado;
            break;
        case PIN_SENSOR_SB1:
            g_sensores_actual.sb1 = estado;
            break;
        case PIN_SENSOR_SB2:
            g_sensores_actual.sb2 = estado;
            break;
        default:
            ESP_LOGW(TAG, "Pin desconocido: %d", pin);
            return;
    }
    
    // Mostrar estado actual de todos los sensores
    ESP_LOGI(TAG, "Estado actual: SA1=%d SA2=%d SB1=%d SB2=%d", 
             g_sensores_actual.sa1, g_sensores_actual.sa2, 
             g_sensores_actual.sb1, g_sensores_actual.sb2);
    
    // Verificar detecciones imposibles
    if (g_sensores_actual.sa1 && g_sensores_actual.sb2) {
        ESP_LOGW(TAG, "ALERTA: Deteccion fisicamente imposible SA1+SB2 - RESETEANDO");
        abrir_barreras_y_resetear();
        return;
    }
    
    // Procesar lógica según estado actual del sistema
    switch (g_sistema.estado) {
        case ESTADO_PASO_PEATONAL:
            procesar_deteccion_aproximacion(&g_sensores_actual);
            break;
            
        case ESTADO_ALERTA_VEHICULO:
            procesar_confirmacion_paso(&g_sensores_actual);
            break;
            
        case ESTADO_PASO_VEHICULAR:
            procesar_paso_completo(&g_sensores_actual);
            procesar_multiples_vehiculos(&g_sensores_actual);
            verificar_apertura_inmediata(&g_sensores_actual);
            break;
    }
}

void procesar_deteccion_aproximacion(sensores_t *sensores) {
    uint64_t tiempo_actual = esp_timer_get_time();
    
    // Detectar aproximacion desde lado A hacia B
    if (sensores->sa1 && !sensores->sa2 && 
        g_sistema.sentido_actual == SENTIDO_NINGUNO) {
        
        ESP_LOGI(TAG, "DETECCION VALIDA: Vehiculo aproximandose A hacia B");
        
        controlar_advertencia_peatonal(true);
        g_sistema.sentido_actual = SENTIDO_A_HACIA_B;
        g_sistema.estado = ESTADO_ALERTA_VEHICULO;
        g_sistema.timestamp_inicio_advertencia = tiempo_actual;
        
        iniciar_timer_inactividad();
        return;
    }
    
    // Detectar aproximacion desde lado B hacia A  
    if (sensores->sb2 && !sensores->sb1 && 
        g_sistema.sentido_actual == SENTIDO_NINGUNO) {
        
        ESP_LOGI(TAG, "DETECCION VALIDA: Vehiculo aproximandose B hacia A");
        
        controlar_advertencia_peatonal(true);
        g_sistema.sentido_actual = SENTIDO_B_HACIA_A;
        g_sistema.estado = ESTADO_ALERTA_VEHICULO;
        g_sistema.timestamp_inicio_advertencia = tiempo_actual;
        
        iniciar_timer_inactividad();
        return;
    }
    
    // Detectar patrones extraños
    if (sensores->sa2 && !sensores->sa1) {
        ESP_LOGW(TAG, "PATRON EXTRANO: SA2 activo sin SA1");
    }
    
    if (sensores->sb1 && !sensores->sb2) {
        ESP_LOGW(TAG, "PATRON EXTRANO: SB1 activo sin SB2");
    }
}

void procesar_confirmacion_paso(sensores_t *sensores) {
    uint64_t tiempo_actual = esp_timer_get_time();
    uint64_t tiempo_advertencia = tiempo_actual - g_sistema.timestamp_inicio_advertencia;
    
    // Confirmacion A hacia B
    if (g_sistema.sentido_actual == SENTIDO_A_HACIA_B && sensores->sa2) {
        
        if (tiempo_advertencia >= TIEMPO_RETARDO_SEGURIDAD) {
            ESP_LOGI(TAG, "CONFIRMACION: Cerrando barreras para paso A hacia B");
            
            enviar_pulso_barrera(false);
            g_sistema.vehiculo_en_transito = true;
            g_sistema.vehiculo_paso_completo = false;
            g_sistema.estado = ESTADO_PASO_VEHICULAR;
            
            iniciar_timer_cierre(TIEMPO_CIERRE_DEFECTO);
            iniciar_timer_inactividad();
        } else {
            ESP_LOGD(TAG, "Esperando tiempo minimo de advertencia...");
        }
        return;
    }
    
    // Confirmacion B hacia A
    if (g_sistema.sentido_actual == SENTIDO_B_HACIA_A && sensores->sb1) {
        
        if (tiempo_advertencia >= TIEMPO_RETARDO_SEGURIDAD) {
            ESP_LOGI(TAG, "CONFIRMACION: Cerrando barreras para paso B hacia A");
            
            enviar_pulso_barrera(false);
            g_sistema.vehiculo_en_transito = true;
            g_sistema.vehiculo_paso_completo = false;
            g_sistema.estado = ESTADO_PASO_VEHICULAR;
            
            iniciar_timer_cierre(TIEMPO_CIERRE_DEFECTO);
            iniciar_timer_inactividad();
        } else {
            ESP_LOGD(TAG, "Esperando tiempo minimo de advertencia...");
        }
        return;
    }
}

void procesar_paso_completo(sensores_t *sensores) {
    // Vehiculo completo paso A hacia B
    if (g_sistema.vehiculo_en_transito && 
        g_sistema.sentido_actual == SENTIDO_A_HACIA_B &&
        sensores->sb1 && sensores->sb2) {
        
        ESP_LOGI(TAG, "OPTIMIZACION: Vehiculo salio por B - Reduciendo timer");
        g_sistema.vehiculo_paso_completo = true;
        iniciar_timer_cierre(TIEMPO_CIERRE_OPTIMIZADO);
        iniciar_timer_inactividad();
        return;
    }
    
    // Vehiculo completo paso B hacia A  
    if (g_sistema.vehiculo_en_transito && 
        g_sistema.sentido_actual == SENTIDO_B_HACIA_A &&
        sensores->sa2 && sensores->sa1) {
        
        ESP_LOGI(TAG, "OPTIMIZACION: Vehiculo salio por A - Reduciendo timer");
        g_sistema.vehiculo_paso_completo = true;
        iniciar_timer_cierre(TIEMPO_CIERRE_OPTIMIZADO);
        iniciar_timer_inactividad();
        return;
    }
}

void procesar_multiples_vehiculos(sensores_t *sensores) {
    if (!g_sistema.vehiculo_en_transito) return;
    
    // Nuevo vehiculo detectado en cola A hacia B
    if (g_sistema.sentido_actual == SENTIDO_A_HACIA_B && sensores->sa1) {
        ESP_LOGI(TAG, "COLA: Nuevo vehiculo detectado en A");
        iniciar_timer_cierre(TIEMPO_CIERRE_DEFECTO);
        g_sistema.vehiculo_paso_completo = false;
        iniciar_timer_inactividad();
        return;
    }
    
    // Nuevo vehiculo detectado en cola B hacia A
    if (g_sistema.sentido_actual == SENTIDO_B_HACIA_A && sensores->sb2) {
        ESP_LOGI(TAG, "COLA: Nuevo vehiculo detectado en B");
        iniciar_timer_cierre(TIEMPO_CIERRE_DEFECTO);
        g_sistema.vehiculo_paso_completo = false;
        iniciar_timer_inactividad();
        return;
    }
    
    // Optimizacion para multiples vehiculos
    if (!g_sistema.vehiculo_paso_completo) {
        if (g_sistema.sentido_actual == SENTIDO_A_HACIA_B && sensores->sb2) {
            ESP_LOGI(TAG, "MULTIPLES: Primer vehiculo salio por B");
            g_sistema.vehiculo_paso_completo = true;
            iniciar_timer_cierre(TIEMPO_CIERRE_MULTIPLE);
        }
        
        if (g_sistema.sentido_actual == SENTIDO_B_HACIA_A && sensores->sa1) {
            ESP_LOGI(TAG, "MULTIPLES: Primer vehiculo salio por A");
            g_sistema.vehiculo_paso_completo = true;
            iniciar_timer_cierre(TIEMPO_CIERRE_MULTIPLE);
        }
    }
}

bool todos_sensores_inactivos(sensores_t *sensores) {
    return !sensores->sa1 && !sensores->sa2 && !sensores->sb1 && !sensores->sb2;
}

void verificar_apertura_inmediata(sensores_t *sensores) {
    static uint64_t tiempo_sin_sensores = 0;
    uint64_t tiempo_actual = esp_timer_get_time();
    
    if (g_sistema.vehiculo_paso_completo && todos_sensores_inactivos(sensores)) {
        if (tiempo_sin_sensores == 0) {
            tiempo_sin_sensores = tiempo_actual;
        } else if (tiempo_actual - tiempo_sin_sensores >= 2000000) { // 2 segundos
            ESP_LOGI(TAG, "APERTURA INMEDIATA: No hay mas vehiculos");
            abrir_barreras_y_resetear();
            tiempo_sin_sensores = 0;
            return;
        }
    } else {
        tiempo_sin_sensores = 0;
    }
}

// ================================
// FUNCIONES DE CONTROL
// ================================
void abrir_barreras_y_resetear() {
    // Detener timers
    if (timer_cierre_barrera) esp_timer_stop(timer_cierre_barrera);
    if (timer_inactividad) esp_timer_stop(timer_inactividad);
    
    // Enviar pulso para abrir barreras
    enviar_pulso_barrera(true);
    
    // Desactivar advertencia peatonal
    controlar_advertencia_peatonal(false);
    
    // Resetear estado del sistema
    g_sistema.barreras_abiertas = true;
    g_sistema.advertencia_peatonal_activa = false;
    g_sistema.sentido_actual = SENTIDO_NINGUNO;
    g_sistema.vehiculo_en_transito = false;
    g_sistema.vehiculo_paso_completo = false;
    g_sistema.estado = ESTADO_PASO_PEATONAL;
    g_sistema.timestamp_inicio_advertencia = 0;
    g_sistema.timestamp_ultimo_sensor = 0;
    
    ESP_LOGI(TAG, "===== SISTEMA RESETEADO - PASO PEATONAL =====");
}

void iniciar_timer_cierre(uint64_t tiempo_us) {
    if (timer_cierre_barrera) {
        esp_timer_stop(timer_cierre_barrera);
        esp_timer_start_once(timer_cierre_barrera, tiempo_us);
        ESP_LOGI(TAG, "Timer cierre iniciado: %llu segundos", tiempo_us / 1000000);
    }
}

void iniciar_timer_inactividad() {
    if (timer_inactividad) {
        esp_timer_stop(timer_inactividad);
        esp_timer_start_once(timer_inactividad, TIEMPO_INACTIVIDAD_MAX);
        g_sistema.timestamp_ultimo_sensor = esp_timer_get_time();
    }
}

void mostrar_estado_sistema() {
    const char* estados[] = {"PASO_PEATONAL", "ALERTA_VEHICULO", "PASO_VEHICULAR"};
    const char* sentidos[] = {"NINGUNO", "A_hacia_B", "B_hacia_A"};
    
    ESP_LOGI(TAG, "ESTADO ACTUAL:");
    ESP_LOGI(TAG, "   Estado: %s", estados[g_sistema.estado]);
    ESP_LOGI(TAG, "   Sentido: %s", sentidos[g_sistema.sentido_actual]);
    ESP_LOGI(TAG, "   Barreras: %s", g_sistema.barreras_abiertas ? "ABIERTAS" : "CERRADAS");
    ESP_LOGI(TAG, "   Advertencia: %s", g_sistema.advertencia_peatonal_activa ? "ACTIVA" : "INACTIVA");
    ESP_LOGI(TAG, "   Vehiculo en transito: %s", g_sistema.vehiculo_en_transito ? "SI" : "NO");
    ESP_LOGI(TAG, "   Sensores: SA1=%d SA2=%d SB1=%d SB2=%d", 
             g_sensores_actual.sa1, g_sensores_actual.sa2, 
             g_sensores_actual.sb1, g_sensores_actual.sb2);
}

// ================================
// INICIALIZACIÓN
// ================================
int init_sensores_como_botones() {
    // Configurar SA1
    btn_sa1.gpio = PIN_SENSOR_SA1;
    btn_sa1.pressed_level = 0;  // Para sensores NPN (activo en LOW)
    btn_sa1.internal_pull = true;
    btn_sa1.autorepeat = false;
    btn_sa1.callback = on_sensor_callback;

    // Configurar SA2
    btn_sa2.gpio = PIN_SENSOR_SA2;
    btn_sa2.pressed_level = 0;
    btn_sa2.internal_pull = true;
    btn_sa2.autorepeat = false;
    btn_sa2.callback = on_sensor_callback;

    // Configurar SB1
    btn_sb1.gpio = PIN_SENSOR_SB1;
    btn_sb1.pressed_level = 0;
    btn_sb1.internal_pull = true;
    btn_sb1.autorepeat = false;
    btn_sb1.callback = on_sensor_callback;

    // Configurar SB2
    btn_sb2.gpio = PIN_SENSOR_SB2;
    btn_sb2.pressed_level = 0;
    btn_sb2.internal_pull = true;
    btn_sb2.autorepeat = false;
    btn_sb2.callback = on_sensor_callback;

    // Inicializar botones/sensores
    ESP_ERROR_CHECK(button_init(&btn_sa1));
    ESP_ERROR_CHECK(button_init(&btn_sa2));
    ESP_ERROR_CHECK(button_init(&btn_sb1));
    ESP_ERROR_CHECK(button_init(&btn_sb2));

    ESP_LOGI(TAG, "Sensores configurados como botones correctamente");
    return 0;
}

void init_timers() {
    // Timer para cierre de barrera
    esp_timer_create_args_t timer_cierre_args = {
        .callback = &callback_cierre_barrera,
        .name = "timer_cierre"
    };
    esp_timer_create(&timer_cierre_args, &timer_cierre_barrera);
    
    // Timer para inactividad
    esp_timer_create_args_t timer_inactividad_args = {
        .callback = &callback_inactividad,
        .name = "timer_inactividad"  
    };
    esp_timer_create(&timer_inactividad_args, &timer_inactividad);
}

// ================================
// TASK DE MONITOREO
// ================================
void task_monitoreo_sistema(void *pvParameters) {
    int contador = 0;
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Cada 10 segundos
        contador++;
        
        ESP_LOGI(TAG, "Heartbeat #%d - Sistema funcionando", contador);
        
        // Mostrar estado cada minuto
        if (contador % 6 == 0) {
            mostrar_estado_sistema();
        }
        
        // Verificar apertura inmediata en caso de estado vehicular
        if (g_sistema.estado == ESTADO_PASO_VEHICULAR) {
            verificar_apertura_inmediata(&g_sensores_actual);
        }
    }
}

void app_main() {
    ESP_LOGI(TAG, "Iniciando sistema de control vehicular con libreria de botones");
    
    // Inicializar NVS (requerido por la librería)
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
    
    // Resetear sistema
    abrir_barreras_y_resetear();
    
    // Crear task de monitoreo
    xTaskCreate(task_monitoreo_sistema, "monitoreo", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Sistema inicializado - Usando libreria de botones para sensores");
    mostrar_estado_sistema();
}