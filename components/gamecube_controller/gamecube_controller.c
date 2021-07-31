#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gamecube_controller.h"
#include <string.h>
#include <sys/time.h>

#define RMT_CLOCK_SPEED 80000000
#define RMT_CLOCK_DIVIDER 80 // This evaluates to 1 RMT tick = 1 microsecond
#define RMT_RX_IDLE_THRESHOLD_US 9500 // Idle threshold for the remote receiver

static gamecube_rx_config* rx_config = NULL;
static rmt_item32_t CONSOLE_TO_CONTROLLER_DATA[25];

// Why does this differ from the documentation?
static uint32_t CONSOLE_TO_CONTROLLER_COMMAND = 0b010000000000001100000000;

static void populate_command_data(uint32_t data, uint8_t num_bits, bool enable_rumble) {
    if (num_bits > 24) {
        printf("CONSOLE_TO_CONTROLLER_DATA bits is higher than 24\n");
        return;
    }

    rmt_item32_t zero_bit = {
        .duration0 = 3, // 3 microseconds low
        .level0 = 0,
        .duration1 = 1, // 1 microsecond high
        .level1 = 1,
    };

    rmt_item32_t one_bit = {
        .duration0 = 1, // 1 microseocond low
        .level0 = 0,
        .duration1 = 3, // 3 microseconds high
        .level1 = 1,
    };


    int bit_counter = num_bits - 1;

    // Iterate through the bits, MSB to LSB.
    for (int i = 0; i < num_bits; i++) {
        bool bit = ((data >> bit_counter) & 1) == 1;
        bit_counter -= 1;

        if (bit) {
            CONSOLE_TO_CONTROLLER_DATA[i] = one_bit;
        } else {
            CONSOLE_TO_CONTROLLER_DATA[i] = zero_bit;
        }
    }

    if (enable_rumble) {
        CONSOLE_TO_CONTROLLER_DATA[num_bits-1] = one_bit;
    }

    // Stop bit
    CONSOLE_TO_CONTROLLER_DATA[num_bits] = one_bit;
}

// Reads a byte from the GameCube controller pulse data.
static uint8_t read_byte(rmt_item32_t* items) {
    uint8_t val = 0;

    for (int i = 0; i < 8; i++) {
        bool is_one = items[i].duration0 == 1;
        val |= (is_one << (7 - i));
    }

    return val;
}

// Writes 8 bytes of controller data to dst, used for sending over the network.
void write_controller_bytes(controller_data* controller, uint8_t* dst) {
    uint8_t byte_0 = 0;
    uint8_t bit_counter = 0;

    byte_0 |= (controller->start_button << bit_counter++);
    byte_0 |= (controller->y_button << bit_counter++);
    byte_0 |= (controller->x_button << bit_counter++);
    byte_0 |= (controller->b_button << bit_counter++);
    byte_0 |= (controller->a_button << bit_counter++);
    byte_0 |= (controller->l_button << bit_counter++);
    byte_0 |= (controller->r_button << bit_counter++);
    byte_0 |= (controller->z_button << bit_counter++);

    uint8_t byte_1 = 0;
    bit_counter = 0;
    byte_1 |= (controller->dpad_up_button << bit_counter++);
    byte_1 |= (controller->dpad_down_button << bit_counter++);
    byte_1 |= (controller->dpad_right_button << bit_counter++);
    byte_1 |= (controller->dpad_left_button << bit_counter++);

    dst[0] = byte_0;
    dst[1] = byte_1;
    dst[2] = controller->joystick_x;
    dst[3] = controller->joystick_y;
    dst[4] = controller->c_stick_x;
    dst[5] = controller->c_stick_y;
    dst[6] = controller->l_bumper;
    dst[7] = controller->r_bumper;
}

// Reads GameCube controller data from a series of pulses from the RMT hardware module.
void controller_from_pulses(rmt_item32_t* pulses, controller_data* controller) {
    controller->start_button = pulses[28].duration0 == 1;
    controller->y_button = pulses[29].duration0 == 1;
    controller->x_button = pulses[30].duration0 == 1;
    controller->b_button = pulses[31].duration0 == 1;
    controller->a_button = pulses[32].duration0 == 1;

    controller->l_button = pulses[34].duration0 == 1;
    controller->r_button = pulses[35].duration0 == 1;
    controller->z_button = pulses[36].duration0 == 1;
    controller->dpad_up_button = pulses[37].duration0 == 1;
    controller->dpad_down_button = pulses[38].duration0 == 1;
    controller->dpad_right_button = pulses[39].duration0 == 1;
    controller->dpad_left_button = pulses[40].duration0 == 1;

    controller->joystick_x = read_byte(&pulses[41]);
    controller->joystick_y = read_byte(&pulses[49]);
    controller->c_stick_x = read_byte(&pulses[57]);
    controller->c_stick_y = read_byte(&pulses[65]);
    controller->l_bumper = read_byte(&pulses[73]);
    controller->r_bumper = read_byte(&pulses[81]);
}

// Reads 8 bytes of controller data from src, which was received over the network.
void controller_from_bytes(uint8_t* src, controller_data* controller) {
    uint8_t bit_counter = 0;

    controller->start_button = (1 << bit_counter++) & (src[0] == 1);
    controller->y_button = (1 << bit_counter++) & (src[0] == 1);
    controller->x_button = (1 << bit_counter++) & (src[0] == 1);
    controller->b_button = (1 << bit_counter++) & (src[0] == 1);
    controller->a_button = (1 << bit_counter++) & (src[0] == 1);
    controller->l_button = (1 << bit_counter++) & (src[0] == 1);
    controller->r_button = (1 << bit_counter++) & (src[0] == 1);
    controller->z_button = (1 << bit_counter++) & (src[0] == 1);

    bit_counter = 0;

    controller->dpad_up_button = (1 << bit_counter++) & (src[1] == 1);
    controller->dpad_down_button = (1 << bit_counter++) & (src[1] == 1);
    controller->dpad_right_button = (1 << bit_counter++) & (src[1] == 1);
    controller->dpad_left_button = (1 << bit_counter++) & (src[1] == 1);

    controller->joystick_x = src[2];
    controller->joystick_y = src[3];
    controller->c_stick_x = src[4];
    controller->c_stick_y = src[5];
    controller->l_bumper = src[6];
    controller->r_bumper = src[7];
}

void print_controller_data(controller_data* controller) {
    printf("S: %u, Y: %u, X: %u, B: %u, A: %u, L: %u, R: %u, Z: %u, ⬆️: %u, ⬇️: %u, ➡️: %u, ⬅️: %u, Joystick: (%u, %u), C-Stick: (%u, %u), Bumps: (%u, %u)\n",
        controller->start_button,
        controller->y_button,
        controller->x_button,
        controller->b_button,
        controller->a_button,
        controller->l_button,
        controller->r_button,
        controller->z_button,
        controller->dpad_up_button,
        controller->dpad_down_button,
        controller->dpad_right_button,
        controller->dpad_left_button,
        controller->joystick_x,
        controller->joystick_y,
        controller->c_stick_x,
        controller->c_stick_y,
        controller->l_bumper,
        controller->r_bumper
    );
}

static void gamecube_rx_task() {
    int tx_channel = 0; // for ESP32-C3, 0-1 are valid TX channels
    int rx_channel = 2; // for ESP32-C3, 2-3 are valid RX channels

    // References for 1-wire implementation for GameCube data protocol:
    // https://github.com/espressif/esp-idf/issues/5237
    // https://github.com/espressif/esp-idf/issues/4608

    // Max ticks per item = 32,768 (rmt_item32_t durations are in ticks and use 15 bits)

    // clock divide = 80
    // 1MHz = 0.000001 seconds per tick
    // 1 tick = 1000 nanoseconds
    // 10 ticks = 10000 nanoseconds
    // max us per item = 32.77 ms

    uint16_t clock_ticks_per_10_us = (RMT_CLOCK_SPEED / RMT_CLOCK_DIVIDER) / 100000;
    // uint16_t ns_per_tick = 1000000000 / (RMT_CLOCK_SPEED / RMT_CLOCK_DIVIDER);
    // uint16_t us_per_tick = 1000000 / (RMT_CLOCK_SPEED / RMT_CLOCK_DIVIDER);
    // uint16_t clock_ticks_per_s = (RMT_CLOCK_SPEED / RMT_CLOCK_DIVIDER);
    uint16_t clock_ticks_per_us = (RMT_CLOCK_SPEED / RMT_CLOCK_DIVIDER) / 1000000;

    if (clock_ticks_per_10_us == 0) {
        printf("clock_ticks_per_10_us is 0, can't start GameCube controller receiver\n");
        vTaskDelete(NULL);
        return;
    }

    printf("RX: clock_ticks_per_10_us: %u\n", clock_ticks_per_10_us);
    printf("RX: clock_ticks_per_us: %u\n", clock_ticks_per_us);

    uint32_t max_us_per_item = ((uint32_t)0x7FFF * 10) /
                               clock_ticks_per_10_us; // (2^15 ticks * 10) / clock_ticks_per_10_us
    printf("RX: max_us_per_item: %u\n", max_us_per_item);

    // TX Setup
    rmt_config_t rmt_tx;

    rmt_tx.rmt_mode = RMT_MODE_TX;
    rmt_tx.channel = tx_channel;
    rmt_tx.gpio_num = rx_config->input_pin;
    rmt_tx.clk_div = RMT_CLOCK_DIVIDER;
    rmt_tx.mem_block_num = 1;
    rmt_tx.flags = 0;

    rmt_tx.tx_config.carrier_freq_hz = 24000000; // Where did this come from?
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    rmt_tx.tx_config.carrier_en = false;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.idle_output_en = true;

    rmt_config(&rmt_tx);
    rmt_driver_install(tx_channel, 0, 0);
    // End TX Setup

    // RX Setup
    rmt_config_t rmt_rx;

    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.channel = rx_channel;
    rmt_rx.gpio_num = rx_config->input_pin;
    rmt_rx.clk_div = RMT_CLOCK_DIVIDER;
    rmt_rx.mem_block_num = 4;
    rmt_rx.flags = 0;

    rmt_rx.rx_config.idle_threshold = RMT_RX_IDLE_THRESHOLD_US / 10 * (clock_ticks_per_10_us);
    rmt_rx.rx_config.filter_ticks_thresh = 0;
    rmt_rx.rx_config.filter_en = false;

    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, rx_config->ring_buffer_size, 0);

    RingbufHandle_t rx_ring_buffer = NULL;
    rmt_get_ringbuf_handle(rx_channel, &rx_ring_buffer);
    // End RX Setup

    // Enable open drain on the pin.
    gpio_set_direction(rx_config->input_pin, GPIO_MODE_INPUT_OUTPUT);
    gpio_matrix_out(rx_config->input_pin, RMT_SIG_OUT0_IDX + tx_channel, 0, 0);
    gpio_matrix_in(rx_config->input_pin, RMT_SIG_IN0_IDX + rx_channel, 0);

    // Populate the "console" command.
    bool enable_rumble = false;
    populate_command_data(CONSOLE_TO_CONTROLLER_COMMAND, 24, enable_rumble);

    size_t cmd_size = sizeof(CONSOLE_TO_CONTROLLER_DATA) / sizeof(CONSOLE_TO_CONTROLLER_DATA[0]);

    // Listen for the controller's response.
    rmt_rx_start(rx_channel, 1);

    while (true) {
        // Send the polling command to the controller.
        bool wait_tx_done = true;
        rmt_write_items(tx_channel, CONSOLE_TO_CONTROLLER_DATA, cmd_size, wait_tx_done);

        size_t rx_size = 0;

        rmt_item32_t* item =
            (rmt_item32_t*)xRingbufferReceive(rx_ring_buffer, &rx_size, 100);

        if (item) {
            size_t num_items = rx_size / sizeof(rmt_item32_t);

            // Item includes both the console command and the controller response, as the RX
            // channel is always listening and the protocol shares one wire.
            if (num_items >= 90) {
                controller_data data;

                controller_from_pulses(&item[0], &data);
                print_controller_data(&data);
            }

            vRingbufferReturnItem(rx_ring_buffer, (void*)item);
        } else {
            // No response received, try again.
        }

        // vTaskDelay(4); // TODO(bschwind) - Double check this value.
    }

    vTaskDelete(NULL);
}

esp_err_t gamecube_rx_start(gamecube_rx_config config) {
    rx_config = malloc(sizeof(gamecube_rx_config));
    memcpy(rx_config, &config, sizeof(gamecube_rx_config));

    xTaskCreate(gamecube_rx_task, "gamecube_rx_task", 1024 * 2, NULL, 10, NULL);

    return ESP_OK;
}
