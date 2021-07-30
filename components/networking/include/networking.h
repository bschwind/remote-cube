#ifndef __NETWORKING_H__
#define __NETWORKING_H__

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Begin receiving data from the gamecube controller
void networking_init();

#ifdef __cplusplus
}
#endif

#endif // __NETWORKING_H__
