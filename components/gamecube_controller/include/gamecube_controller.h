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
    // Digital Buttons
    bool start_button;
    bool y_button;
    bool x_button;
    bool b_button;
    bool a_button;
    bool l_button;
    bool r_button;
    bool z_button;
    bool dpad_up_button;
    bool dpad_down_button;
    bool dpad_right_button;
    bool dpad_left_button;

    // Analog Values
    uint8_t joystick_x;
    uint8_t joystick_y;
    uint8_t c_stick_x;
    uint8_t c_stick_y;
    uint8_t l_bumper;
    uint8_t r_bumper;
} controller_data;

typedef struct {
    gpio_num_t input_pin;
    xQueueHandle gamecube_data_queue;
    size_t ring_buffer_size;
} gamecube_rx_config;

// Begin receiving data from the gamecube controller
esp_err_t gamecube_rx_start(gamecube_rx_config config);

// Writes 8 bytes of controller data to dst, used for sending over the network.
void write_controller_bytes(controller_data* controller, uint8_t* dst);

// Reads 8 bytes of controller data from src, which was received over the network.
void controller_from_bytes(uint8_t* src, controller_data* controller);

// Reads GameCube controller data from a series of pulses from the RMT hardware module.
void controller_from_pulses(rmt_item32_t* pulses, controller_data* controller);

// Prints out the controller info on the serial port.
void print_controller_data(controller_data* controller);

#ifdef __cplusplus
}
#endif

#endif // __GAMECUBE_CONTROLLER_H__
