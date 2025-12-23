#include "kincony_board.h"

esp_err_t kincony_init(i2c_master_bus_handle_t bus, kincony_board_t *board) {
    i2c_device_config_t cfg_in = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = KINCONY_ADDR_INPUTS, .scl_speed_hz = 100000 };
    i2c_device_config_t cfg_out = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = KINCONY_ADDR_RELAYS, .scl_speed_hz = 100000 };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &cfg_in, &board->input_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &cfg_out, &board->relay_handle));
    return ESP_OK;
}

esp_err_t kincony_read_inputs(kincony_board_t *board, uint8_t *val) {
    uint8_t data;
    esp_err_t ret = i2c_master_receive(board->input_handle, &data, 1, -1);
    if (ret == ESP_OK) {
        // Invertimos porque el opto tira a GND (0) cuando hay 12V
        *val = (~data) & 0x3F; 
    }
    return ret;
}

esp_err_t kincony_write_relays(kincony_board_t *board, uint8_t state_mask) {
    // Los relés en estas placas suelen activarse con nivel BAJO (inverted: true en tu YAML)
    uint8_t data = ~state_mask; 
    return i2c_master_transmit(board->relay_handle, &data, 1, -1);
}