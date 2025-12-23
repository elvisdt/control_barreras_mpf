#ifndef KINCONY_BOARD_H
#define KINCONY_BOARD_H

#include "driver/i2c_master.h"

#define KINCONY_ADDR_INPUTS  0x22  // Según tu ESPHome
#define KINCONY_ADDR_RELAYS  0x24  // Según tu ESPHome

typedef struct {
    i2c_master_dev_handle_t input_handle;
    i2c_master_dev_handle_t relay_handle;
} kincony_board_t;

esp_err_t kincony_init(i2c_master_bus_handle_t bus, kincony_board_t *board);
esp_err_t kincony_read_inputs(kincony_board_t *board, uint8_t *val);
esp_err_t kincony_write_relays(kincony_board_t *board, uint8_t state_mask);

#endif