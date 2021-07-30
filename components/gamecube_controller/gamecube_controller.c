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

static uint16_t us_to_ticks(uint16_t us, uint16_t clock_ticks_per_10_us) {
    return ((us * clock_ticks_per_10_us) / 10) & 0x7FFF;
}

static uint16_t ticks_to_us(uint16_t ticks, uint16_t clock_ticks_per_10_us) {
    return (((ticks & 0x7FFF) * 10) / clock_ticks_per_10_us);
}

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
    rmt_rx.mem_block_num = 1;
    rmt_rx.flags = 0;

    rmt_rx.rx_config.idle_threshold = RMT_RX_IDLE_THRESHOLD_US;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.filter_en = true;

    // rmt_config(&rmt_rx);
    // rmt_driver_install(rmt_rx.channel, rx_config->ring_buffer_size, 0);

    // RingbufHandle_t rx_ring_buffer = NULL;
    // rmt_get_ringbuf_handle(rx_channel, &rx_ring_buffer);
    // End RX Setup


    bool enable_rumble = true;
    populate_command_data(CONSOLE_TO_CONTROLLER_COMMAND, 24, enable_rumble);

    // Basic logic:
    // while not connected:
    //     Send the probe data (000000001) every 12ms
    // Once connected:
    //     1.) Clear the RX memory and call rmt_rx_stop()
    //     Send the data request (0100 0000 0000 0011 0000 0010) as a blocking call.
    //     Call rmt_rx_start()
    //     Receive the data from the ring buffer with xRingbufferReceive().
    //     If no data is received, go to the "while not connected" loop above.
    //     Send the received data over the controller data queue
    //     Sleep for some amount of time (6ms?)
    //     Go to step 1

    bool controller_connected = false;
    size_t cmd_size = sizeof(CONSOLE_TO_CONTROLLER_DATA) / sizeof(CONSOLE_TO_CONTROLLER_DATA[0]);

    while (true) {
        while (!controller_connected) {
            // Stop trying to receive data.
            // rmt_rx_stop(rx_channel);

            // Send the polling command to the controller.
            bool wait_tx_done = true;
            rmt_write_items(tx_channel, CONSOLE_TO_CONTROLLER_DATA, cmd_size, wait_tx_done);

            // Listen for the controller's response.
            // rmt_rx_start(rx_channel, 1);

            // size_t rx_size = 0;

            // rmt_item32_t* item =
            //     (rmt_item32_t*)xRingbufferReceive(rx_ring_buffer, &rx_size, 6 / portTICK_RATE_MS);

            // if (item) {
            //     // TODO(bschwind) - Only set this to true if we got a valid response.
            //     controller_connected = true;

            //     size_t num_items = rx_size / sizeof(rmt_item32_t);
            //     printf("rx_size: %u bytes, %u items\n", rx_size, num_items);

            //     for (int i = 0; i < num_items; i++) {
            //         uint16_t duration0 = ticks_to_us(item[i].duration0, clock_ticks_per_10_us);
            //         uint16_t duration1 = ticks_to_us(item[i].duration1, clock_ticks_per_10_us);

            //         printf("duration0 (microseconds): %u, duration1 (microseconds): %u\n", duration0, duration1);
            //     }

            //     vRingbufferReturnItem(rx_ring_buffer, (void*)item);
            // } else {
            //     // No response received in 6 ms, try again.
            // }
        }

        // Stop trying to receive data.
        // rmt_rx_stop(rx_channel);

        // Send the polling command to the controller.
        bool wait_tx_done = true;
        rmt_write_items(tx_channel, CONSOLE_TO_CONTROLLER_DATA, cmd_size, wait_tx_done);

        // TODO - read the response

        vTaskDelay(10 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

esp_err_t gamecube_rx_start(gamecube_rx_config config) {
    rx_config = malloc(sizeof(gamecube_rx_config));
    memcpy(rx_config, &config, sizeof(gamecube_rx_config));

    xTaskCreate(gamecube_rx_task, "gamecube_rx_task", 1024 * 2, NULL, 10, NULL);

    return ESP_OK;
}
