#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gamecube_controller.h"
#include "networking.h"

#define GAMECUBE_IN_PIN 0
#define IS_SERVER CONFIG_ESP_SERVER_MODE

void app_main(void) {
    QueueHandle_t queue = xQueueCreate(10, sizeof(controller_data));
    networking_init(queue);

    if (IS_SERVER) {
        // TODO: gamecube_tx
    } else {
        gamecube_rx_config rx_config = {
            .input_pin = GAMECUBE_IN_PIN,
            .gamecube_data_queue = queue,
            .ring_buffer_size = 3000};
        esp_err_t err = gamecube_rx_start(rx_config);
    }


    if (err) {
        printf("GameCube rx start failed: %i\n", err);
        return;
    }
}
