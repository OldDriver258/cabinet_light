#pragma once

#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * Maximum number of light channels
 */
#define LIGHT_MAX               (6)
/**
 * @brief Light module initialization
 * @return Error code
 */
esp_err_t light_module_init(void);
/**
 * @brief Start the light control of the corresponding channel
 * @param channel Light channel
 * @return Error code
 */
esp_err_t light_control_start(int channel);
/**
 * @brief Stop the light control of the corresponding channel
 * @param channel Light channel
 * @return Error code
 */
esp_err_t light_control_stop(int channel);

#ifdef __cplusplus
}
#endif