#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gamecube_controller.h"
#include "networking.h"

#define GAMECUBE_IN_PIN CONFIG_CUBE_DATA_PIN

void app_main(void) {
    QueueHandle_t queue = xQueueCreate(1, sizeof(controller_data));
    networking_init(queue);

    gamecube_rx_config rx_config = {
        .input_pin = GAMECUBE_IN_PIN, .gamecube_data_queue = queue, .ring_buffer_size = 3000};
    esp_err_t err;

#if CONFIG_CUBE_SENDER
    printf("GameCube Sender Mode - Capture packets from a controller and send them over the "
           "network.\n");
    err = gamecube_rx_start(rx_config);
#elif CONFIG_CUBE_RECEIVER
    printf("GameCube Receiver Mode - Receive packets from a remote ESP32 and play them back to a "
           "local console.\n");
    err = gamecube_tx_start(rx_config);
#endif

    if (err) {
        printf("GameCube rx start failed: %i\n", err);
        return;
    }
}
