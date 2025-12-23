#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "kincony_board.h" // Asegúrate de que este archivo esté en tu proyecto

#define I2C_SDA 4
#define I2C_SCL 15

static const char *TAG = "KINCONY_MAIN";
static i2c_master_bus_handle_t bus_handle;

// Inicialización del Bus I2C
void i2c_bus_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
}

// Función de escaneo para verificar hardware
void i2c_scan(i2c_master_bus_handle_t bus) {
    ESP_LOGI(TAG, "Escaneando bus I2C...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        esp_err_t ret = i2c_master_probe(bus, addr, 100);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Dispositivo encontrado en: 0x%02X", addr);
        }
    }
}

// void app_main(void) {
//     // 1. Inicializar Hardware
//     i2c_bus_init();
    
//     // 2. Escaneo rápido (opcional, útil para depurar)
//     i2c_scan(bus_handle);

//     // 3. Configurar la librería de entradas Kincony
//     kincony_inputs_t board;
//     // Usamos 0x24 porque es lo que detectó tu escaneo anterior
//     esp_err_t ret = kincony_inputs_init(bus_handle, 0x24, &board);

//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "No se pudo inicializar el PCF8574. Revisa conexiones.");
//         return;
//     }

//     // 4. Bucle de lectura
//     while (1) {
//         uint8_t bits = 0;
//         ret = kincony_inputs_read(&board, &bits);
        
//         if ( ret == ESP_OK) {
//             // Imprimimos el estado de las 6 entradas del esquema
//             ESP_LOGI(TAG,"ESTADO DI -> [1]:%d [2]:%d [3]:%d [4]:%d [5]:%d [6]:%d\r",
//                     (bits >> 0) & 1, (bits >> 1) & 1, (bits >> 2) & 1,
//                     (bits >> 3) & 1, (bits >> 4) & 1, (bits >> 5) & 1);
//             fflush(stdout); 
//         }else
//         {
//             ESP_LOGE(TAG,"ret: %d",ret);
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(100)); // 10 lecturas por segundo
//     }
// }


void app_main(void) {
    i2c_bus_init(); 
    i2c_scan(bus_handle);
    kincony_board_t board;
    kincony_init(bus_handle, &board);

    while (1) {
        uint8_t inputs = 0;
        if (kincony_read_inputs(&board, &inputs) == ESP_OK) {
            // Ejemplo: El relé 1 sigue el estado de la entrada 1
            kincony_write_relays(&board, inputs); 
            
            ESP_LOGI(TAG,"Entradas activas: 0x%02X", inputs);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}