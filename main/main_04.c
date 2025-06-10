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

static const char *TAG = "CTRL";

// ================================
// CONFIGURACIÓN DE PINES
// ================================
#define PIN_SENSOR_SA1              GPIO_NUM_4   // Sensor A1 (más alejado)
#define PIN_SENSOR_SA2              GPIO_NUM_18   // Sensor A2 (más cerca)
#define PIN_SENSOR_SB1              GPIO_NUM_19   // Sensor B1 (más cerca)
#define PIN_SENSOR_SB2              GPIO_NUM_21    // Sensor B2 (más alejado)

#define PIN_BARRERA_ABRIR           GPIO_NUM_22   // Pulso para abrir barrera
#define PIN_BARRERA_CERRAR          GPIO_NUM_23   // Pulso para cerrar barrera
#define PIN_ADVERTENCIA_PEATONAL    GPIO_NUM_2  // Luces + sirena advertencia peatonal

// Configuración de pulsos
#define DURACION_PULSO_MS      500          // Duración del pulso en ms

// Configuración de sensores (modificar según tipo de sensor LOOP)
#define USAR_PULL_UPS_INTERNOS  1           // 1=Habilitar, 0=Deshabilitar
#define INVERTIR_LOGICA_SENSOR  1           // 1=Invertir (NPN), 0=Directo (PNP)

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
    bool barreras_abiertas;           // Estado lógico (no controlamos directamente)
    bool advertencia_peatonal_activa; // Luces + sirena
    sentido_t sentido_actual;
    bool vehiculo_en_transito;
    bool vehiculo_paso_completo;
    estado_sistema_t estado;
    uint32_t timestamp_inicio_advertencia;
    uint32_t timestamp_ultimo_sensor;
} sistema_estado_t;

// ================================
// VARIABLES GLOBALES
// ================================
static sistema_estado_t g_sistema = {0};
static sensores_t g_sensores_actual = {0};
static esp_timer_handle_t timer_cierre_barrera = NULL;
static esp_timer_handle_t timer_inactividad = NULL;

// Cola para eventos de sensores
static QueueHandle_t cola_eventos_sensores = NULL;

// Estructura para eventos de sensores
typedef struct {
    gpio_num_t pin;
    bool estado;
    uint64_t timestamp;
} evento_sensor_t;

// Configuración de tiempos (en microsegundos)
#define TIEMPO_RETARDO_SEGURIDAD    (5 * 1000000)  // 5 segundos
#define TIEMPO_CIERRE_DEFECTO       (30 * 1000000) // 30 segundos  
#define TIEMPO_CIERRE_OPTIMIZADO    (5 * 1000000)  // 5 segundos
#define TIEMPO_CIERRE_MULTIPLE      (10 * 1000000) // 10 segundos
#define TIEMPO_INACTIVIDAD_MAX      (30 * 1000000) // 30 segundos
#define TIEMPO_MAX_CERRADO          (120 * 1000000) // 2 minutos

// ================================
// DECLARACIONES DE FUNCIONES
// ================================
void abrir_barreras_y_resetear(void);
void enviar_pulso_barrera(bool abrir);
void controlar_advertencia_peatonal(bool activar);
void iniciar_timer_cierre(uint64_t tiempo_us);
void iniciar_timer_inactividad(void);
void mostrar_estado_sistema(void);
void reset_manual_sistema(void);
void limpiar_estado_inicial_sensores(void);
void verificar_sensores_pegados(void);
void procesar_deteccion_aproximacion(sensores_t *sensores);
void procesar_confirmacion_paso(sensores_t *sensores);
void procesar_paso_completo(sensores_t *sensores);
void procesar_multiples_vehiculos(sensores_t *sensores);
void verificar_apertura_inmediata(sensores_t *sensores);
void actualizar_estado_sensor(gpio_num_t pin, bool estado);
bool todos_sensores_inactivos(sensores_t *sensores);
// FALTAN estas declaraciones:
void diagnostico_avanzado_sensores(void);
void procesar_evento_sensor(evento_sensor_t *evento);
// ================================
// FUNCIONES DE HARDWARE
// ================================

// ISR para sensores (se ejecuta en contexto de interrupción)
static void IRAM_ATTR isr_sensor_handler(void* arg) {
    gpio_num_t pin = (gpio_num_t) arg;
    bool estado_raw = gpio_get_level(pin);
    
    // Crear evento (aplicar inversión según configuración)
    evento_sensor_t evento = {
        .pin = pin,
        .estado = INVERTIR_LOGICA_SENSOR ? !estado_raw : estado_raw,
        .timestamp = esp_timer_get_time()
    };
    
    // Enviar a cola (desde ISR)
    BaseType_t higher_priority_task_woken = pdFALSE;
    xQueueSendFromISR(cola_eventos_sensores, &evento, &higher_priority_task_woken);
    
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

void init_gpio() {
    // Crear cola para eventos de sensores
    cola_eventos_sensores = xQueueCreate(20, sizeof(evento_sensor_t));
    
    // Configurar sensores como entrada con pull-up configurable
    gpio_config_t sensor_config = {
        .pin_bit_mask = (1ULL << PIN_SENSOR_SA1) | (1ULL << PIN_SENSOR_SA2) |
                       (1ULL << PIN_SENSOR_SB1) | (1ULL << PIN_SENSOR_SB2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = USAR_PULL_UPS_INTERNOS ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE  // Interrupción en flanco ascendente y descendente
    };
    gpio_config(&sensor_config);

    // Configurar salidas para pulsos y advertencia
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
    
    // Instalar servicio de interrupciones GPIO
    gpio_install_isr_service(0);
    
    // Asociar ISR a cada sensor
    gpio_isr_handler_add(PIN_SENSOR_SA1, isr_sensor_handler, (void*) PIN_SENSOR_SA1);
    gpio_isr_handler_add(PIN_SENSOR_SA2, isr_sensor_handler, (void*) PIN_SENSOR_SA2);
    gpio_isr_handler_add(PIN_SENSOR_SB1, isr_sensor_handler, (void*) PIN_SENSOR_SB1);
    gpio_isr_handler_add(PIN_SENSOR_SB2, isr_sensor_handler, (void*) PIN_SENSOR_SB2);
    
    ESP_LOGI(TAG, "GPIO e interrupciones configuradas correctamente");
    ESP_LOGI(TAG, "Configuracion: Pull-ups=%s, Inversion=%s", 
             USAR_PULL_UPS_INTERNOS ? "SI" : "NO",
             INVERTIR_LOGICA_SENSOR ? "SI" : "NO");
}

void actualizar_estado_sensor(gpio_num_t pin, bool estado) {
    switch (pin) {
        case PIN_SENSOR_SA1:  // GPIO_NUM_21
            g_sensores_actual.sa1 = estado;
            break;
        case PIN_SENSOR_SA2:  // GPIO_NUM_19
            g_sensores_actual.sa2 = estado;
            break;
        case PIN_SENSOR_SB1:  // GPIO_NUM_18
            g_sensores_actual.sb1 = estado;
            break;
        case PIN_SENSOR_SB2:  // GPIO_NUM_4
            g_sensores_actual.sb2 = estado;
            break;
        default:
            ESP_LOGW(TAG, "Pin desconocido en actualizar_estado_sensor: %d", pin);
            break;
    }
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
    
    // RESETEAR COMPLETAMENTE EL ESTADO DEL SISTEMA
    g_sistema.barreras_abiertas = true;
    g_sistema.advertencia_peatonal_activa = false;
    g_sistema.sentido_actual = SENTIDO_NINGUNO;
    g_sistema.vehiculo_en_transito = false;
    g_sistema.vehiculo_paso_completo = false;
    g_sistema.estado = ESTADO_PASO_PEATONAL;
    g_sistema.timestamp_inicio_advertencia = 0;
    g_sistema.timestamp_ultimo_sensor = 0;
    
    ESP_LOGI(TAG, "===== SISTEMA COMPLETAMENTE RESETEADO - PASO PEATONAL =====");
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

// ================================
// LÓGICA PRINCIPAL DEL SISTEMA
// ================================
void procesar_deteccion_aproximacion(sensores_t *sensores) {
    uint64_t tiempo_actual = esp_timer_get_time();
    
    // Detectar aproximacion desde lado A hacia B
    // SA1 debe activarse PRIMERO y SA2 debe estar inactivo
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
    // SB2 debe activarse PRIMERO y SB1 debe estar inactivo
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
    
    // Detectar patrones invalidos y resetear si es necesario
    if (sensores->sa2 && !sensores->sa1) {
        ESP_LOGW(TAG, "PATRON EXTRANO: SA2 activo sin SA1 - Posible vehiculo ya en la zona");
    }
    
    if (sensores->sb1 && !sensores->sb2) {
        ESP_LOGW(TAG, "PATRON EXTRANO: SB1 activo sin SB2 - Posible vehiculo ya en la zona");
    }
}

void procesar_confirmacion_paso(sensores_t *sensores) {
    uint64_t tiempo_actual = esp_timer_get_time();
    uint64_t tiempo_advertencia = tiempo_actual - g_sistema.timestamp_inicio_advertencia;
    
    // Confirmacion A hacia B
    if (g_sistema.sentido_actual == SENTIDO_A_HACIA_B && sensores->sa2) {
        
        if (tiempo_advertencia >= TIEMPO_RETARDO_SEGURIDAD) {
            ESP_LOGI(TAG, "CONFIRMACION: Cerrando barreras para paso A hacia B");
            
            enviar_pulso_barrera(false);  // Enviar pulso para cerrar
            g_sistema.vehiculo_en_transito = true;
            g_sistema.vehiculo_paso_completo = false;
            g_sistema.estado = ESTADO_PASO_VEHICULAR;
            
            iniciar_timer_cierre(TIEMPO_CIERRE_DEFECTO);
            iniciar_timer_inactividad();
        } else {
            ESP_LOGD(TAG, "Esperando tiempo minimo de advertencia... (%llu ms restantes)", 
                    (TIEMPO_RETARDO_SEGURIDAD - tiempo_advertencia) / 1000);
        }
        return;
    }
    
    // Confirmacion B hacia A
    if (g_sistema.sentido_actual == SENTIDO_B_HACIA_A && sensores->sb1) {
        
        if (tiempo_advertencia >= TIEMPO_RETARDO_SEGURIDAD) {
            ESP_LOGI(TAG, "CONFIRMACION: Cerrando barreras para paso B hacia A");
            
            enviar_pulso_barrera(false);  // Enviar pulso para cerrar
            g_sistema.vehiculo_en_transito = true;
            g_sistema.vehiculo_paso_completo = false;
            g_sistema.estado = ESTADO_PASO_VEHICULAR;
            
            iniciar_timer_cierre(TIEMPO_CIERRE_DEFECTO);
            iniciar_timer_inactividad();
        } else {
            ESP_LOGD(TAG, "Esperando tiempo minimo de advertencia... (%llu ms restantes)", 
                    (TIEMPO_RETARDO_SEGURIDAD - tiempo_advertencia) / 1000);
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
    
    // Nuevo vehiculo detectado en cola A hacia B (solo cuando SA1 se activa)
    if (g_sistema.sentido_actual == SENTIDO_A_HACIA_B && sensores->sa1) {
        ESP_LOGI(TAG, "COLA: Nuevo vehiculo detectado en A");
        iniciar_timer_cierre(TIEMPO_CIERRE_DEFECTO);  // Reset a tiempo completo
        g_sistema.vehiculo_paso_completo = false;
        iniciar_timer_inactividad();
        return;
    }
    
    // Nuevo vehiculo detectado en cola B hacia A (solo cuando SB2 se activa)
    if (g_sistema.sentido_actual == SENTIDO_B_HACIA_A && sensores->sb2) {
        ESP_LOGI(TAG, "COLA: Nuevo vehiculo detectado en B");
        iniciar_timer_cierre(TIEMPO_CIERRE_DEFECTO);  // Reset a tiempo completo
        g_sistema.vehiculo_paso_completo = false;
        iniciar_timer_inactividad();
        return;
    }
    
    // Optimizacion para multiples vehiculos - primer vehiculo salio
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

void limpiar_estado_inicial_sensores() {
    ESP_LOGI(TAG, "Limpiando estado inicial de sensores...");
    
    // Resetear estado de sensores a inactivo
    g_sensores_actual.sa1 = false;
    g_sensores_actual.sa2 = false;
    g_sensores_actual.sb1 = false;
    g_sensores_actual.sb2 = false;
    
    // Limpiar la cola de eventos para empezar fresco
    xQueueReset(cola_eventos_sensores);
    
    ESP_LOGI(TAG, "Estado de sensores limpiado - Esperando eventos reales...");
}

void verificar_sensores_pegados() {
    // Leer estado real de hardware RAW (sin inversión)
    int sa1_raw = gpio_get_level(PIN_SENSOR_SA1);
    int sa2_raw = gpio_get_level(PIN_SENSOR_SA2);
    int sb1_raw = gpio_get_level(PIN_SENSOR_SB1);
    int sb2_raw = gpio_get_level(PIN_SENSOR_SB2);
    
    // Estado procesado (con inversión para pull-up)
    bool sa1_hw = INVERTIR_LOGICA_SENSOR ? !sa1_raw : sa1_raw;
    bool sa2_hw = INVERTIR_LOGICA_SENSOR ? !sa2_raw : sa2_raw;
    bool sb1_hw = INVERTIR_LOGICA_SENSOR ? !sb1_raw : sb1_raw;
    bool sb2_hw = INVERTIR_LOGICA_SENSOR ? !sb2_raw : sb2_raw;

    ESP_LOGI(TAG, "=== DIAGNOSTICO DE HARDWARE ===");
    ESP_LOGI(TAG, "Estado RAW (directo del GPIO): SA1=%d SA2=%d SB1=%d SB2=%d", 
             sa1_raw, sa2_raw, sb1_raw, sb2_raw);
    ESP_LOGI(TAG, "Estado PROCESADO (invertido): SA1=%d SA2=%d SB1=%d SB2=%d", 
             sa1_hw, sa2_hw, sb1_hw, sb2_hw);
    
    // Analizar patrones de problema
    if (sa1_raw == 1 && sa2_raw == 1 && sb1_raw == 1 && sb2_raw == 1) {
        ESP_LOGW(TAG, "TODOS GPIO EN HIGH (1)");
    } else if (sa1_raw == 0 && sa2_raw == 0 && sb1_raw == 0 && sb2_raw == 0) {
        ESP_LOGW(TAG, "TODOS GPIO EN LOW (0)");
    }
    
    // Si todos los sensores procesados estan activos, limpiar estado
    if (sa1_hw && sa2_hw && sb1_hw && sb2_hw) {
        ESP_LOGW(TAG, "TODOS LOS SENSORES PROCESADOS ACTIVOS - Limpiando estado");
        limpiar_estado_inicial_sensores();
        return;
    }
    
    // Actualizar estado real
    g_sensores_actual.sa1 = sa1_hw;
    g_sensores_actual.sa2 = sa2_hw;
    g_sensores_actual.sb1 = sb1_hw;
    g_sensores_actual.sb2 = sb2_hw;
    
    ESP_LOGI(TAG, "Estado inicial actualizado desde hardware");
}

void diagnostico_avanzado_sensores() {
    ESP_LOGI(TAG, "=== DIAGNOSTICO AVANZADO ===");
    
    // Test de configuración actual
    ESP_LOGI(TAG, "Configuracion actual:");
    ESP_LOGI(TAG, "- Pull-ups internos: %s", USAR_PULL_UPS_INTERNOS ? "HABILITADOS" : "DESHABILITADOS");
    ESP_LOGI(TAG, "- Inversion logica: %s", INVERTIR_LOGICA_SENSOR ? "HABILITADA (NPN)" : "DESHABILITADA (PNP)");
    ESP_LOGI(TAG, "- Tipo interrupcion: ANYEDGE");
    
    // Lectura múltiple para detectar fluctuaciones
    ESP_LOGI(TAG, "Leyendo sensores ...");
    // for (int i = 0; i < 5; i++) {
        // Leer valores RAW del hardware
        int sa1_raw = gpio_get_level(PIN_SENSOR_SA1);
        int sa2_raw = gpio_get_level(PIN_SENSOR_SA2);
        int sb1_raw = gpio_get_level(PIN_SENSOR_SB1);
        int sb2_raw = gpio_get_level(PIN_SENSOR_SB2);
        
        // Aplicar lógica de procesamiento
        bool sa1_proc = INVERTIR_LOGICA_SENSOR ? !sa1_raw : sa1_raw;
        bool sa2_proc = INVERTIR_LOGICA_SENSOR ? !sa2_raw : sa2_raw;
        bool sb1_proc = INVERTIR_LOGICA_SENSOR ? !sb1_raw : sb1_raw;
        bool sb2_proc = INVERTIR_LOGICA_SENSOR ? !sb2_raw : sb2_raw;
        
        ESP_LOGI(TAG, "Lectura PROC: SA1=%d SA2=%d SB1=%d SB2=%d", 
                sa1_proc, sa2_proc, sb1_proc, sb2_proc);
        
        vTaskDelay(pdMS_TO_TICKS(200));
    // }
    ESP_LOGI(TAG, "=== FIN DIAGNOSTICO ===");
}

void reset_manual_sistema() {
    ESP_LOGW(TAG, "RESET MANUAL SOLICITADO");
    abrir_barreras_y_resetear();
    mostrar_estado_sistema();
}


void procesar_evento_sensor(evento_sensor_t *evento) {
    // Debounce: ignorar cambios muy rápidos (menos de 50ms)
    static int last_idx=0;
    static uint64_t ultimo_evento[4] = {0};
    static gpio_num_t pines_sensores[] = {PIN_SENSOR_SA1, PIN_SENSOR_SA2, PIN_SENSOR_SB1, PIN_SENSOR_SB2};
    
    int indice_pin = -1;
    for (int i = 0; i < 4; i++) {
        if (pines_sensores[i] == evento->pin) {
            indice_pin = i;
            break;
        }
    }
    
    if (indice_pin >= 0) {
        uint64_t tiempo_desde_ultimo = evento->timestamp - ultimo_evento[indice_pin];
        if (tiempo_desde_ultimo < 50000) { // 50ms debounce
            ESP_LOGD(TAG, "Debounce: ignorando evento en pin %d", evento->pin);
            return;
        }
        ultimo_evento[indice_pin] = evento->timestamp;
    }
    
    // Actualizar estado del sensor
    actualizar_estado_sensor(evento->pin, evento->estado);
    
    // Log del cambio con más detalle
    const char* nombres_pines[] = {"SA1", "SA2", "SB1", "SB2"};
    const char* nombre_pin = "DESCONOCIDO";
    if (indice_pin >= 0) {
        nombre_pin = nombres_pines[indice_pin];
    }
    
    ESP_LOGI(TAG, "Sensor %s: %s (Estado actual: SA1=%d SA2=%d SB1=%d SB2=%d)", 
             nombre_pin, evento->estado ? "ACTIVADO" : "DESACTIVADO",
             g_sensores_actual.sa1, g_sensores_actual.sa2, 
             g_sensores_actual.sb1, g_sensores_actual.sb2);
    
    // Verificar detecciones imposibles (fisicamente imposible que esten activos a la vez)
    if (g_sensores_actual.sa1 && g_sensores_actual.sb2) {
        ESP_LOGW(TAG, "ALERTA: Deteccion fisicamente imposible SA1+SB2 - IGNORANDO y RESETEANDO");
        abrir_barreras_y_resetear();
        return;
    }
    
    // Verificar otros patrones sospechosos
    if (g_sensores_actual.sa2 && g_sensores_actual.sb1) {
        ESP_LOGW(TAG, "PATRON SOSPECHOSO: SA2+SB1 activos simultaneamente");
    }
    
    // Solo procesar si estamos en estado valido
    if (g_sistema.estado == ESTADO_PASO_PEATONAL && 
        (g_sensores_actual.sa1 || g_sensores_actual.sa2 || g_sensores_actual.sb1 || g_sensores_actual.sb2)) {
        ESP_LOGI(TAG, "Estado: PASO_PEATONAL - Analizando deteccion...");
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
        
    if (indice_pin != last_idx) {
        switch (last_idx)
        {
        case 0:// PIN_SENSOR_SA1
            g_sensores_actual.sa1 = 0;
            break;
        case 1:// PIN_SENSOR_SA2
            g_sensores_actual.sa2 = 0;
            break;
        case 2:// PIN_SENSOR_SB1
            g_sensores_actual.sb1 = 0;
            break;
        case 3:// PIN_SENSOR_SB2
            g_sensores_actual.sb2 = 0;
            break;
        default:
            break;
        }
        last_idx = indice_pin;
    }
}

// ================================
// TASK PRINCIPAL
// ================================
void task_control_vehicular(void *pvParameters) {
    evento_sensor_t evento;
    int contador_heartbeat = 0;
    
    ESP_LOGI(TAG, "Sistema de control vehicular iniciado con interrupciones");
    abrir_barreras_y_resetear();
    
    // Leer estado inicial de sensores (con lógica configurable)
    bool sa1_raw = gpio_get_level(PIN_SENSOR_SA1);
    bool sa2_raw = gpio_get_level(PIN_SENSOR_SA2);
    bool sb1_raw = gpio_get_level(PIN_SENSOR_SB1);
    bool sb2_raw = gpio_get_level(PIN_SENSOR_SB2);
    
    g_sensores_actual.sa1 = INVERTIR_LOGICA_SENSOR ? !sa1_raw : sa1_raw;  // GPIO 21
    g_sensores_actual.sa2 = INVERTIR_LOGICA_SENSOR ? !sa2_raw : sa2_raw;  // GPIO 19
    g_sensores_actual.sb1 = INVERTIR_LOGICA_SENSOR ? !sb1_raw : sb1_raw;  // GPIO 18
    g_sensores_actual.sb2 = INVERTIR_LOGICA_SENSOR ? !sb2_raw : sb2_raw;  // GPIO 4
    
    ESP_LOGI(TAG, "Estado inicial sensores: SA1=%d SA2=%d SB1=%d SB2=%d", 
             g_sensores_actual.sa1, g_sensores_actual.sa2, 
             g_sensores_actual.sb1, g_sensores_actual.sb2);
    
    // Si todos los sensores están activos, ejecutar diagnóstico avanzado
    if (g_sensores_actual.sa1 && g_sensores_actual.sa2 && g_sensores_actual.sb1 && g_sensores_actual.sb2) {
        ESP_LOGW(TAG, "TODOS LOS SENSORES ACTIVOS - Ejecutando diagnostico avanzado...");
        diagnostico_avanzado_sensores();
    }
    
    // Verificar si hay sensores pegados y limpiar estado si es necesario
    verificar_sensores_pegados();
    mostrar_estado_sistema();
    
    while (1) {
        
        memset(&evento, 0, sizeof(evento_sensor_t));

        // Esperar eventos de sensores (bloqueante con timeout)
        if (xQueueReceive(cola_eventos_sensores, &evento, pdMS_TO_TICKS(5000)) == pdTRUE) {
            // Procesar evento de sensor
            ESP_LOGE(TAG,"EVNT: PIN [%02d], stado: [%d], time: %lld",evento.pin, evento.estado, evento.timestamp);
            // procesar_evento_sensor(&evento);

        } else {
            // Timeout cada 5 segundos - mostrar heartbeat
            contador_heartbeat++;
            ESP_LOGI(TAG, "Heartbeat #%d - Sistema funcionando", contador_heartbeat);
            
            // Mostrar estado cada 10 heartbeats (50 segundos) si hay actividad
            if (contador_heartbeat % 20 == 0 && g_sistema.estado != ESTADO_PASO_PEATONAL) {
                mostrar_estado_sistema();
            }
            
            // Verificar si hay algun problema de sensores pegados
            bool sensores_activos = g_sensores_actual.sa1 || g_sensores_actual.sa2 || 
                                   g_sensores_actual.sb1 || g_sensores_actual.sb2;
            
            if (sensores_activos && g_sistema.estado == ESTADO_PASO_PEATONAL) {
                ESP_LOGW(TAG, "Sensores activos en estado peatonal - Verificar hardware");
                mostrar_estado_sistema();
                
                // Cada 10 heartbeats (50 segundos) ejecutar diagnóstico completo
                if (contador_heartbeat % 10 == 0) {
                    ESP_LOGW(TAG, "Re-ejecutando diagnostico completo...");
                    diagnostico_avanzado_sensores();
                    verificar_sensores_pegados();
                }
            }
            
            // Verificar si necesita apertura inmediata (sin eventos)
            if (g_sistema.estado == ESTADO_PASO_VEHICULAR) {
                verificar_apertura_inmediata(&g_sensores_actual);
            }
        }
    }
}

// ================================
// INICIALIZACIÓN
// ================================
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

void app_main() {
    ESP_LOGI(TAG, "Iniciando sistema de control vehicular y peatonal con interrupciones");
    
    // Inicializar hardware y timers
    init_gpio();
    init_timers();
    vTaskDelay(pdMS_TO_TICKS(5000)); // Esperar 1 segundo para estabilizar
    // Crear task principal con mayor prioridad para responsividad
    xTaskCreate(task_control_vehicular, "control_vehicular", 1024*8, NULL, 6, NULL);
    
    ESP_LOGI(TAG, "Sistema inicializado correctamente - Usando interrupciones GPIO");
}