#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gamecube_controller.h"
#include "networking.h"

#define GAMECUBE_IN_PIN 0
#define IS_SERVER 1

void app_main(void) {
    QueueHandle_t queue = xQueueCreate(10, sizeof(controller_data));
    networking_init(queue);

    gamecube_rx_config rx_config = {
        .input_pin = GAMECUBE_IN_PIN,
        .gamecube_data_queue = queue,
        .ring_buffer_size = 3000};
    esp_err_t err;
    if (IS_SERVER) {
        printf("GameCube TRANSMIT MODE.\n");
        err = gamecube_tx_start(rx_config);
    } else {
        printf("GameCube RECEIVE MODE.\n");
        err = gamecube_rx_start(rx_config);
    }
    if (err) {
        printf("GameCube rx start failed: %i\n", err);
        return;
    }


}
