#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gamecube_controller.h"
#include <string.h>
#include <sys/time.h>

#define RMT_CLOCK_SPEED 80000000
#define RMT_CLOCK_DIVIDER 80 // This evaluates to 1 RMT tick = 1 microsecond
#define RMT_RX_IDLE_THRESHOLD_US 9500 // Idle threshold for the remote receiver

static gamecube_rx_config* rx_config = NULL;

static const rmt_item32_t zero_bit = {
    .duration0 = 3, // 3 microseconds low
    .level0 = 0,
    .duration1 = 1, // 1 microsecond high
    .level1 = 1,
};

static const rmt_item32_t one_bit = {
    .duration0 = 1, // 1 microseocond low
    .level0 = 0,
    .duration1 = 3, // 3 microseconds high
    .level1 = 1,
};

// 01000000 00000011 00000000 - 0x40, 0x03, 0x00
static rmt_item32_t CONSOLE_TO_CONTROLLER_DATA[25] = {
    zero_bit,
    one_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,

    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    one_bit,
    one_bit,

    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,

    one_bit
};

// 00001001 00000000 00000011 - 0x09, 0x00, 0x03
static rmt_item32_t CONTROLLER_PROBE_RESPONSE[25] = {
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    one_bit,
    zero_bit,
    zero_bit,
    one_bit,

    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,

    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    zero_bit,
    one_bit,
    one_bit,

    one_bit
};

// Reads a byte from the GameCube controller pulse data.
static inline uint8_t IRAM_ATTR read_byte(rmt_item32_t* items) {
    uint8_t val = 0;

    for (int i = 0; i < 8; i++) {
        bool is_one = items[i].duration0 < items[i].duration1;
        val |= (is_one << (7 - i));
        // printf("%u|%d %u|%d\n", items[i].level0, items[i].duration0, items[i].level1, items[i].duration1);
    }

    // printf("Read byte: %u\n", val);

    return val;
}

// Reads a byte from the GameCube controller pulse data.
static inline void IRAM_ATTR write_byte(uint8_t b, rmt_item32_t* items) {
    // printf("write_byte(%d, ", b);
    for (int i = 0; i < 8; i++) {
        bool is_one = ((b >> (7-i)) & 1) == 1;
        items[i] = is_one ? one_bit : zero_bit;
        // printf("%d", is_one);
    }
    // printf(")\n");
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

// Reads 8 bytes of controller data from src, which was received over the network.
void controller_from_bytes(uint8_t* src, controller_data* controller) {
    uint8_t bit_counter = 0;

    controller->start_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->y_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->x_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->b_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->a_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->l_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->r_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->z_button = ((1 << bit_counter++) & src[0]) != 0;

    bit_counter = 0;

    controller->dpad_up_button = ((1 << bit_counter++) & src[1]) != 0;
    controller->dpad_down_button = ((1 << bit_counter++) & src[1]) != 0;
    controller->dpad_right_button = ((1 << bit_counter++) & src[1]) != 0;
    controller->dpad_left_button = ((1 << bit_counter++) & src[1]) != 0;

    controller->joystick_x = src[2];
    controller->joystick_y = src[3];
    controller->c_stick_x = src[4];
    controller->c_stick_y = src[5];
    controller->l_bumper = src[6];
    controller->r_bumper = src[7];
}

// Reads GameCube controller data from a series of pulses from the RMT hardware module.
void controller_from_pulses(rmt_item32_t* pulses, controller_data* controller) {
    controller->start_button = pulses[28].duration0 < pulses[28].duration1;
    controller->y_button = pulses[29].duration0 < pulses[29].duration1;
    controller->x_button = pulses[30].duration0 < pulses[30].duration1;
    controller->b_button = pulses[31].duration0 < pulses[31].duration1;
    controller->a_button = pulses[32].duration0 < pulses[32].duration1;

    controller->l_button = pulses[34].duration0 < pulses[34].duration1;
    controller->r_button = pulses[35].duration0 < pulses[35].duration1;
    controller->z_button = pulses[36].duration0 < pulses[36].duration1;
    controller->dpad_up_button = pulses[37].duration0 < pulses[37].duration1;
    controller->dpad_down_button = pulses[38].duration0 < pulses[38].duration1;
    controller->dpad_right_button = pulses[39].duration0 < pulses[39].duration1;
    controller->dpad_left_button = pulses[40].duration0 < pulses[40].duration1;

    controller->joystick_x = read_byte(&pulses[41]);
    controller->joystick_y = read_byte(&pulses[49]);
    controller->c_stick_x = read_byte(&pulses[57]);
    controller->c_stick_y = read_byte(&pulses[65]);
    controller->l_bumper = read_byte(&pulses[73]);
    controller->r_bumper = read_byte(&pulses[81]);
}

void IRAM_ATTR controller_to_pulses(controller_data* controller, rmt_item32_t* pulses) {
    pulses[0] = zero_bit;
    pulses[1] = zero_bit;
    pulses[2] = zero_bit;
    pulses[3] = controller->start_button ? one_bit : zero_bit;
    pulses[4] = controller->y_button ? one_bit : zero_bit;
    pulses[5] = controller->x_button ? one_bit : zero_bit;
    pulses[6] = controller->b_button ? one_bit : zero_bit;
    pulses[7] = controller->a_button ? one_bit : zero_bit;
    pulses[8] = one_bit;
    pulses[9] = controller->l_button ? one_bit : zero_bit;
    pulses[10] = controller->r_button ? one_bit : zero_bit;
    pulses[11] = controller->z_button ? one_bit : zero_bit;
    pulses[12] = controller->dpad_up_button   ? one_bit : zero_bit;
    pulses[13] = controller->dpad_down_button ? one_bit : zero_bit;
    pulses[14] = controller->dpad_right_button ? one_bit : zero_bit;
    pulses[15] = controller->dpad_left_button ? one_bit : zero_bit;

    write_byte(controller->joystick_x, &pulses[16]);
    write_byte(controller->joystick_y, &pulses[24]);
    write_byte(controller->c_stick_x, &pulses[32]);
    write_byte(controller->c_stick_y, &pulses[40]);
    write_byte(controller->l_bumper, &pulses[48]);
    write_byte(controller->r_bumper, &pulses[56]);
    pulses[64] = one_bit;
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
    rmt_rx.mem_block_num = 2;
    rmt_rx.flags = 0;

    rmt_rx.rx_config.idle_threshold = RMT_RX_IDLE_THRESHOLD_US / 10 * (clock_ticks_per_10_us);
    rmt_rx.rx_config.filter_ticks_thresh = 0;
    rmt_rx.rx_config.filter_en = false;

    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, rx_config->ring_buffer_size, 0);

    RingbufHandle_t rx_ring_buffer = NULL;
    rmt_get_ringbuf_handle(rx_channel, &rx_ring_buffer);
    // End RX Setup

    // Set the pin to both input and output mode, but not open drain.
    gpio_set_direction(rx_config->input_pin, GPIO_MODE_INPUT_OUTPUT);
    gpio_matrix_out(rx_config->input_pin, RMT_SIG_OUT0_IDX + tx_channel, 0, 0);
    gpio_matrix_in(rx_config->input_pin, RMT_SIG_IN0_IDX + rx_channel, 0);

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

                xQueueSendToBack(rx_config->gamecube_data_queue, &data, (TickType_t)0);
            }

            vRingbufferReturnItem(rx_ring_buffer, (void*)item);
        } else {
            // No response received, try again.
        }
    }

    vTaskDelete(NULL);
}

static void gamecube_tx_task() {
    int tx_channel = 0; // for ESP32-C3, 0-1 are valid TX channels
    int rx_channel = 2; // for ESP32-C3, 2-3 are valid RX channels

    uint16_t clock_ticks_per_us = (RMT_CLOCK_SPEED / RMT_CLOCK_DIVIDER) / 1000000;
    printf("RX: clock_ticks_per_us: %u\n", clock_ticks_per_us);

    // TX Setup
    rmt_config_t rmt_tx;

    rmt_tx.rmt_mode = RMT_MODE_TX;
    rmt_tx.channel = tx_channel;
    rmt_tx.gpio_num = rx_config->input_pin;
    rmt_tx.clk_div = RMT_CLOCK_DIVIDER;
    rmt_tx.mem_block_num = 1;
    rmt_tx.flags = 0;

    rmt_tx.tx_config.idle_level = 1;
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
    rmt_rx.mem_block_num = 2;
    rmt_rx.flags = 0;

    rmt_rx.rx_config.idle_threshold = 6;
    rmt_rx.rx_config.filter_ticks_thresh = 0;
    rmt_rx.rx_config.filter_en = false;

    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, rx_config->ring_buffer_size, 0);

    RingbufHandle_t rx_ring_buffer = NULL;
    rmt_get_ringbuf_handle(rx_channel, &rx_ring_buffer);
    // End RX Setup

    // Enable open drain on the pin.
    gpio_set_direction(rx_config->input_pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_matrix_out(rx_config->input_pin, RMT_SIG_OUT0_IDX + tx_channel, 0, 0);
    gpio_matrix_in(rx_config->input_pin, RMT_SIG_IN0_IDX + rx_channel, 0);

    // Listen for the controller's response.
    rmt_rx_start(rx_channel, true);

    controller_data controller_msg;
    rmt_item32_t out_pulses[81];

    while (true) {
        size_t rx_size = 0;
        rmt_item32_t* item =
            (rmt_item32_t*)xRingbufferReceive(rx_ring_buffer, &rx_size, 100);

        if (item) {
            size_t num_items = rx_size / sizeof(rmt_item32_t);
            // printf("rmt num_items %d\n", num_items);

            if (num_items == 9) {
                uint8_t msg = read_byte(item);

                if (msg == 0x00 || msg == 0xff) {
                    // The console is probing for a controller, write a controller probe response.
                    rmt_write_items(tx_channel, &CONTROLLER_PROBE_RESPONSE[0], 25, true);

                    // Now is a good time to fetch the latest controller data from the queue.
                    while (xQueueReceive(rx_config->gamecube_data_queue, (void *)&controller_msg, 0)) {
                        // Just exhausing the queue so we have the latest data.
                    }
                } else if (msg == 0x41 || msg == 0x42) {
                    // The console is probing for the controller origin data (where the analog values sit when not touched).
                    // We may not want to reply here until we've received at least one packet of controller data.
                    controller_msg.start_button = 0;
                    controller_msg.y_button = 0;
                    controller_msg.x_button = 0;
                    controller_msg.b_button = 0;
                    controller_msg.a_button = 0;
                    controller_msg.l_button = 0;
                    controller_msg.r_button = 0;
                    controller_msg.z_button = 0;
                    controller_msg.dpad_up_button = 0;
                    controller_msg.dpad_down_button = 0;
                    controller_msg.dpad_right_button = 0;
                    controller_msg.dpad_left_button = 0;
                    controller_to_pulses(&controller_msg, &out_pulses[0]);
                    write_byte(0x00, &out_pulses[64]);
                    write_byte(0x00, &out_pulses[72]);
                    out_pulses[80] = one_bit;

                    rmt_write_items(tx_channel, &out_pulses[0], 81, true);
                }
            } else if (num_items == 25) {
                uint8_t msg1 = read_byte(&item[0]);
                uint8_t msg2 = read_byte(&item[8]);
                // uint8_t msg3 = read_byte(&item[16]);

                if (msg1 == 0x40 && msg2 == 0x03) {
                    // The console is requesting controller data.
                    // The LSB in msg3 is a boolean indicating whether or not
                    // the controller should rumble.

                    // Get the next data to send to the controller.
                    xQueueReceive(rx_config->gamecube_data_queue, (void *)&controller_msg, 0);
                    controller_to_pulses(&controller_msg, &out_pulses[0]);
                    rmt_write_items(tx_channel, &out_pulses[0], 65, true);
                }
            }

            vRingbufferReturnItem(rx_ring_buffer, (void*)item);
        } else {
            // No response received, try again.
        }
    }

    vTaskDelete(NULL);
}

esp_err_t gamecube_rx_start(gamecube_rx_config config) {
    rx_config = malloc(sizeof(gamecube_rx_config));
    memcpy(rx_config, &config, sizeof(gamecube_rx_config));

    xTaskCreate(gamecube_rx_task, "gamecube_rx_task", 1024 * 2, NULL, 10, NULL);

    return ESP_OK;
}

esp_err_t gamecube_tx_start(gamecube_rx_config config) {
    rx_config = malloc(sizeof(gamecube_rx_config));
    memcpy(rx_config, &config, sizeof(gamecube_rx_config));

    xTaskCreatePinnedToCore(gamecube_tx_task, "gamecube_tx_task", 1024 * 4, NULL, 10, NULL, 1);

    return ESP_OK;
}
