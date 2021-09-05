#ifndef __CUBE_WIFI_H__
#define __CUBE_WIFI_H__

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the WiFi network stack.
void cube_wifi_init();

#ifdef __cplusplus
}
#endif

#endif // __CUBE_WIFI_H__
