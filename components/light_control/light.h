#pragma once

#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t register_light_control_cmd(void);
esp_err_t light_ledc_init(void);
esp_err_t light_set_and_update(uint8_t light_ch, uint32_t duty_a, uint32_t duty_b);

#ifdef __cplusplus
}
#endif