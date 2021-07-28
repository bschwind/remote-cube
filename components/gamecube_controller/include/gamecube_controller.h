#ifndef __GAMECUBE_CONTROLLER_H__
#define __GAMECUBE_CONTROLLER_H__

#include "driver/gpio.h"
#include "driver/rmt.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t input_pin;
    xQueueHandle gamecube_data_queue;
    uint16_t idle_threshold_us;
    uint8_t clock_divider;
    size_t ring_buffer_size;
} gamecube_rx_config;

// Begin receiving data from the gamecube controller
esp_err_t gamecube_rx_start(gamecube_rx_config config);

#ifdef __cplusplus
}
#endif

#endif // __GAMECUBE_CONTROLLER_H__
