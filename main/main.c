#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gamecube_controller.h"
#include "networking.h"

#define GAMECUBE_IN_PIN 0

void app_main(void) {
    QueueHandle_t queue = xQueueCreate(10, 8);
    networking_init(false, queue);

    gamecube_rx_config rx_config = {
        .input_pin = GAMECUBE_IN_PIN,
        // TODO - Double check the ring buffer size, we may not need 3000
        .ring_buffer_size = 3000};

    esp_err_t err = gamecube_rx_start(rx_config);

    if (err) {
        printf("GameCube rx start failed: %i\n", err);
        return;
    }

}
