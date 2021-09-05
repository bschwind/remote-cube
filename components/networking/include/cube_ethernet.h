#ifndef __CUBE_ETHERNET_H__
#define __CUBE_ETHERNET_H__

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the ethernet network stack.
void cube_ethernet_init();

#ifdef __cplusplus
}
#endif

#endif // __CUBE_ETHERNET_H__
