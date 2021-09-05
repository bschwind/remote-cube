#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"

#include "cube_ethernet.h"

#define ETH_PHY_ADDR 0
#define ETH_PHY_RST_GPIO (-1)
#define ETH_MDC_GPIO 23
#define ETH_MDIO_GPIO 18
#define PIN_PHY_POWER 12

static const char* TAG = "cube_ethernet";

// FreeRTOS event group to signal when we are connected.
static EventGroupHandle_t ethernet_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define NETWORK_CONNECTED_BIT BIT0
#define NETWORK_FAIL_BIT BIT1

static void eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id,
                              void* event_data) {
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t*)event_data;

    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
            ESP_LOGI(TAG, "Ethernet Link Up");
            ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0],
                     mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Ethernet Link Down");
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Ethernet Started");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGI(TAG, "Ethernet Stopped");
            break;
        default:
            break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id,
                                 void* event_data) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    const esp_netif_ip_info_t* ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");

    xEventGroupSetBits(ethernet_event_group, NETWORK_CONNECTED_BIT);
}

void cube_ethernet_init() {
    ethernet_event_group = xEventGroupCreate();

    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create new default instance of esp-netif for Ethernet
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t* eth_netif = esp_netif_new(&cfg);

    // Set default handlers to process TCP/IP stuffs
    ESP_ERROR_CHECK(esp_eth_set_default_handlers(eth_netif));

    // Register user defined event handers
    ESP_ERROR_CHECK(
        esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(
        esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // Init MAC and PHY configs to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_PHY_RST_GPIO;

    mac_config.smi_mdc_gpio_num = ETH_MDC_GPIO;
    mac_config.smi_mdio_gpio_num = ETH_MDIO_GPIO;

    // Olimex-specific stuff?
    gpio_pad_select_gpio(PIN_PHY_POWER);
    gpio_set_direction(PIN_PHY_POWER, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_PHY_POWER, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set up the MAC.
    esp_eth_mac_t* mac = esp_eth_mac_new_esp32(&mac_config);

    // Olimex boards use the LAN8710 chip, which is compatible with LAN8720.
    esp_eth_phy_t* phy = esp_eth_phy_new_lan8720(&phy_config);

    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));

    // Attach Ethernet driver to TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    // Start Ethernet driver state machine
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    /* Waiting until either the connection is established (NETWORK_CONNECTED_BIT) or connection
     * failed for the maximum number of re-tries (NETWORK_FAIL_BIT). The bits are set by
     * event_handler() (see above) */
    EventBits_t bits =
        xEventGroupWaitBits(ethernet_event_group, NETWORK_CONNECTED_BIT | NETWORK_FAIL_BIT, pdFALSE,
                            pdFALSE, portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which
     * event actually happened. */
    if (bits & NETWORK_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected");
    } else if (bits & NETWORK_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to network");
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
    // instance_got_ip)); ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT,
    // ESP_EVENT_ANY_ID, instance_any_id)); vEventGroupDelete(ethernet_event_group);
}
