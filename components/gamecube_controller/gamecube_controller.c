#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gamecube_controller.h"
#include <string.h>
#include <sys/time.h>

#define RMT_CLOCK_SPEED 80000000

static gamecube_rx_config* rx_config = NULL;

static uint16_t us_to_ticks(uint16_t us, uint16_t clock_ticks_per_10_us) {
    return ((us * clock_ticks_per_10_us) / 10) & 0x7FFF;
}

static uint16_t ticks_to_us(uint16_t ticks, uint16_t clock_ticks_per_10_us) {
    return (((ticks & 0x7FFF) * 10) / clock_ticks_per_10_us);
}

static void gamecube_rx_task() {
    int rx_channel = 0;

    if (rx_config->clock_divider == 0) {
        printf("rx_config.clock_divider is 0, can't start GameCube controller receiver\n");
        vTaskDelete(NULL);
        return;
    }

    // References for 1-wire implementation for GameCube data protocol:
    // https://github.com/espressif/esp-idf/issues/5237
    // https://github.com/espressif/esp-idf/issues/4608

    // Max ticks per item = 32,768 (rmt_item32_t durations are in ticks and use 15 bits)

    // clock divide = 1
    // 80MHz = 0.0000000125 seconds per tick
    // 1 tick = 12.5 nanoseconds
    // 10 ticks = 125 nanoseconds
    // max us per item = 0.41 ms

    // clock divide = 2
    // 40MHz = 0.000000025 seconds per tick
    // 1 tick = 25 nanoseconds
    // 10 ticks = 250 nanoseconds
    // max us per item = 0.82 ms

    // clock divide = 10
    // 8MHz = 0.000000125 seconds per tick
    // 1 tick = 125 nanoseconds
    // 10 ticks = 1250 nanoseconds
    // max us per item = 4.1 ms

    // clock divide = 80
    // 1MHz = 0.000001 seconds per tick
    // 1 tick = 1000 nanoseconds
    // 10 ticks = 10000 nanoseconds
    // max us per item = 32.77 ms

    // ...

    // clock divide = 255
    // 0.313725 MHz = 0.00000318750498 seconds per tick
    // 1 tick = 3190 nanoseconds
    // 10 ticks = 31900 nanoseconds
    // 400 tick = 10 microseconds (10,000 nanoseconds)
    // max us per item = 104.53 ms

    uint16_t clock_ticks_per_10_us = (RMT_CLOCK_SPEED / rx_config->clock_divider) / 100000;
    // uint16_t ns_per_tick = 1000000000 / (RMT_CLOCK_SPEED / rx_config->clock_divider);
    // uint16_t us_per_tick = 1000000 / (RMT_CLOCK_SPEED / rx_config->clock_divider);
    // uint16_t clock_ticks_per_s = (RMT_CLOCK_SPEED / rx_config->clock_divider);
    uint16_t clock_ticks_per_us = (RMT_CLOCK_SPEED / rx_config->clock_divider) / 1000000;

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

    rmt_config_t rmt_rx;

    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.channel = rx_channel;
    rmt_rx.clk_div = rx_config->clock_divider;
    rmt_rx.gpio_num = rx_config->input_pin;
    rmt_rx.mem_block_num = 7; // One block has 64 * 32 bits, so 7 = 1792 bytes

    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    uint64_t idle_threshold_ticks_64 =
        ((rx_config->idle_threshold_us / 10) * clock_ticks_per_10_us);
    printf("idle_threshold_ticks_64: %llu\n", idle_threshold_ticks_64);

    if (idle_threshold_ticks_64 <= UINT16_MAX) {
        rmt_rx.rx_config.idle_threshold = (uint16_t)idle_threshold_ticks_64;
    } else {
        printf("Warning: rmt idle_threshold_us is over uint16_t max\n");
        rmt_rx.rx_config.idle_threshold = UINT16_MAX;
    }

    if (rx_config->idle_threshold_us > max_us_per_item) {
        printf("Warning: rmt idle_threshold_us (%u) is over the maximum microseconds per rmt item "
               "(%u)\n",
               rx_config->idle_threshold_us, max_us_per_item);
    }

    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, rx_config->ring_buffer_size, 0);

    RingbufHandle_t rb = NULL;
    rmt_get_ringbuf_handle(rx_channel, &rb);

    rmt_rx_start(rx_channel, 1);

    struct timeval last_rx_time;
    gettimeofday(&last_rx_time, NULL);

    while (true) {
        size_t rx_size = 0;

        rmt_item32_t* item =
            (rmt_item32_t*)xRingbufferReceive(rb, &rx_size, 10000 / portTICK_RATE_MS);

        struct timeval now, diff;
        gettimeofday(&now, NULL);
        timersub(&now, &last_rx_time, &diff);
        last_rx_time = now;
        printf("Time between captures: %lu sec, %lu us\n", diff.tv_sec, diff.tv_usec);

        size_t num_items = rx_size / sizeof(rmt_item32_t);
        printf("rx_size: %u bytes, %u items\n", rx_size, num_items);

        for (int i = 0; i < num_items; i++) {
            uint16_t duration0 = ticks_to_us(item[i].duration0, clock_ticks_per_10_us);
            uint16_t duration1 = ticks_to_us(item[i].duration1, clock_ticks_per_10_us);

            printf("duration0 (microseconds): %u, duration1 (microseconds): %u\n", duration0, duration1);
        }

        if (item) {
            vRingbufferReturnItem(rb, (void*)item);
        } else {
            printf("Item was NULL, not returning to rx ring buffer\n");
        }

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
