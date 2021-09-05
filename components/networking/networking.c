#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

#include "gamecube_controller.h"

#if CONFIG_CUBE_USE_WIFI
#include "cube_wifi.h"
#elif CONFIG_CUBE_USE_ETHERNET
#include "cube_ethernet.h"
#endif

#if CONFIG_CUBE_SENDER
#define HOST_IP_ADDR CONFIG_CUBE_SERVER_IP
#endif

#define PORT CONFIG_CUBE_SERVER_PORT

static const char* TAG = "cube_networking";

#ifdef CONFIG_CUBE_RECEIVER
static void udp_server_task(void* pvParameters) {
    char rx_buffer[128];
    char addr_str[128];
    QueueHandle_t queue = (QueueHandle_t)pvParameters;
    int addr_family = AF_INET;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in* dest_addr_ip4 = (struct sockaddr_in*)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1) {
            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                               (struct sockaddr*)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            } else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in*)&source_addr)->sin_addr, addr_str,
                                sizeof(addr_str) - 1);
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6*)&source_addr)->sin6_addr, addr_str,
                                 sizeof(addr_str) - 1);
                }

                if (len == 8) {
                    controller_data controller_msg;
                    controller_from_bytes(&rx_buffer, &controller_msg);
                    xQueueOverwrite(queue, (void*)&controller_msg);
                } else {
                    ESP_LOGW(TAG, "wrong len: wanted 8, was %d", len);
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}
#endif

#ifdef CONFIG_CUBE_SENDER
static void udp_client_task(void* pvParameters) {
    QueueHandle_t queue = (QueueHandle_t)pvParameters;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        controller_data controller_msg;
        uint8_t serialized[8] = {0};
        while (1) {
            if (xQueueReceive(queue, (void*)&controller_msg, portMAX_DELAY)) {
                write_controller_bytes(&controller_msg, (uint8_t*)&serialized);
                int err = sendto(sock, &serialized, sizeof(serialized), 0,
                                 (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }

            vTaskDelay(0);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}
#endif

void networking_init(QueueHandle_t packet_queue) {
#if CONFIG_CUBE_USE_WIFI
    cube_wifi_init();
#else CONFIG_CUBE_USE_ETHERNET
    cube_ethernet_init();
#endif

#if CONFIG_CUBE_SENDER
    xTaskCreatePinnedToCore(udp_client_task, "udp_client", 4096, (void*)packet_queue, 5, NULL, 0);
#elif CONFIG_CUBE_RECEIVER
    xTaskCreatePinnedToCore(udp_server_task, "udp_server", 4096, (void*)packet_queue, 5, NULL, 0);
#endif
}
