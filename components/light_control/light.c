#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_check.h"
#include "argtable3/argtable3.h"
#include "light.h"

#define LIGHT_MAX               (6)
#define LIGHT_PWM_MAX           (12)
#define LIGHT_PWM(x)            (light_control_config.pwms[(x-1)])

#define LIGHT_DUTY_RESOLUTION   LEDC_TIMER_12_BIT
#define LIGHT_FREQUENCY         (16000)
#define LIGHT_DUTY_MAX          ((1 << LIGHT_DUTY_RESOLUTION) - 1)
#define LIGHT_PULSE_DELAY       (4)
#define LIGHT_STD_DUTY_MAX      (100)
#define LIGHT_REL_DUTY_MAX      (95)

#define LIGHT_CHECK(a, str, ret_val) ESP_RETURN_ON_FALSE(a, ret_val, MODULE, "%s", str)
#define LIGHT_ARG_CHECK(a, param) ESP_RETURN_ON_FALSE(a, ESP_ERR_INVALID_ARG, MODULE, param " argument is invalid")

static const char* MODULE = "cabinet_light";

struct light_control_pwm {
    ledc_mode_t     speed_mode;
    ledc_timer_t    timer_sel;
    ledc_channel_t  channel;
    int             gpio_num;
};

static struct {
    struct light_control_pwm pwms[LIGHT_PWM_MAX];
} light_control_config = {
    .pwms = {
        {   //PWM1
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0,
            .channel    = LEDC_CHANNEL_0,
            .gpio_num   = 13
        },
        {   //PWM2
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0,
            .channel    = LEDC_CHANNEL_1,
            .gpio_num   = 15
        },
        {   //PWM3
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0,
            .channel    = LEDC_CHANNEL_2,
            .gpio_num   = 2
        },
        {   //PWM4
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0,
            .channel    = LEDC_CHANNEL_3,
            .gpio_num   = 4
        },
        {   //PWM5
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0,
            .channel    = LEDC_CHANNEL_4,
            .gpio_num   = 16
        },
        {   //PWM6
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0,
            .channel    = LEDC_CHANNEL_5,
            .gpio_num   = 17
        },
        {   //PWM7
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0,
            .channel    = LEDC_CHANNEL_6,
            .gpio_num   = 5
        },
        {   //PWM8
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0,
            .channel    = LEDC_CHANNEL_7,
            .gpio_num   = 18
        },
        {   //PWM9
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_1,
            .channel    = LEDC_CHANNEL_0,
            .gpio_num   = 19
        },
        {   //PWM10
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_1,
            .channel    = LEDC_CHANNEL_1,
            .gpio_num   = 21
        },
        {   //PWM11
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_1,
            .channel    = LEDC_CHANNEL_2,
            .gpio_num   = 22
        },
        {   //PWM12
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_1,
            .channel    = LEDC_CHANNEL_3,
            .gpio_num   = 23
        }
    }
};

static struct {
    struct arg_int *channel;
    struct arg_int *pulse_width_a;
    struct arg_int *pulse_width_b;
    struct arg_end *end;
} light_control_args;

static int light_control_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &light_control_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, light_control_args.end, argv[0]);
        return 1;
    }

    return light_set_and_update(light_control_args.channel->ival[0],
                                light_control_args.pulse_width_a->ival[0],
                                light_control_args.pulse_width_b->ival[0]);
}

esp_err_t register_light_control_cmd(void)
{
    light_control_args.channel = 
        arg_int1("c", "channel", "<1-6>", "Select a light channel to control");
    light_control_args.pulse_width_a = 
        arg_int1("a", "pulse_width_a", "<0-100>", "Pulse widths of signal A, A+B <= 100");
    light_control_args.pulse_width_b = 
        arg_int1("b", "pulse_width_b", "<0-100>", "Pulse widths of signal B, A+B <= 100");
    light_control_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command    = "light",
        .help       = "Control light status."
                      "Specify the pulse widths of the two signals AB",
        .hint       = NULL,
        .func       = &light_control_cmd,
        .argtable   = &light_control_args
    };

    return esp_console_cmd_register(&cmd);
}

esp_err_t light_ledc_init(void)
{
    ledc_timer_config_t     ledc_timer;
    ledc_channel_config_t   ledc_channel;
    esp_err_t               err;
    int                     pwm_ch;
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    memset(&ledc_timer, 0, sizeof(ledc_timer));
    ledc_timer.duty_resolution  = LIGHT_DUTY_RESOLUTION,    // resolution of PWM duty
    ledc_timer.freq_hz          = LIGHT_FREQUENCY,          // frequency of PWM signal
    ledc_timer.speed_mode       = LEDC_HIGH_SPEED_MODE,     // timer mode
    ledc_timer.timer_num        = LEDC_TIMER_0,             // timer index
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK,            // Auto select the source clock
    err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "ledc high speed timer init failed.");
        return err;
    }
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_1;
    ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "ledc low speed timer init failed.");
        return err;
    }

    // Prepare and then apply the LEDC PWM channel configuration
    memset(&ledc_channel, 0, sizeof(ledc_channel));
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.duty           = 0;
    ledc_channel.hpoint         = 0;
    for (pwm_ch = 0; pwm_ch < LIGHT_PWM_MAX; pwm_ch++) {
        ledc_channel.speed_mode = light_control_config.pwms[pwm_ch].speed_mode;
        ledc_channel.channel    = light_control_config.pwms[pwm_ch].channel;
        ledc_channel.timer_sel  = light_control_config.pwms[pwm_ch].timer_sel;
        ledc_channel.gpio_num   = light_control_config.pwms[pwm_ch].gpio_num;
        err = ledc_channel_config(&ledc_channel);
        if (err != ESP_OK) {
            ESP_LOGE(MODULE, "pwm channel(%d) init failed.", pwm_ch);
            return err;
        }
    }

    return ESP_OK;
}

// Brightness 0 - 100% gamma correction look up table (gamma = 2.6)
// Y = B ^ 2.6
// Pre-computed LUT to save some runtime computation
static const float gamma_correction_lut[101] = {
    0.000000, 0.000006, 0.000038, 0.000110, 0.000232, 0.000414, 0.000666, 0.000994, 0.001406, 0.001910,
    0.002512, 0.003218, 0.004035, 0.004969, 0.006025, 0.007208, 0.008525, 0.009981, 0.011580, 0.013328,
    0.015229, 0.017289, 0.019512, 0.021902, 0.024465, 0.027205, 0.030125, 0.033231, 0.036527, 0.040016,
    0.043703, 0.047593, 0.051688, 0.055993, 0.060513, 0.065249, 0.070208, 0.075392, 0.080805, 0.086451,
    0.092333, 0.098455, 0.104821, 0.111434, 0.118298, 0.125416, 0.132792, 0.140428, 0.148329, 0.156498,
    0.164938, 0.173653, 0.182645, 0.191919, 0.201476, 0.211321, 0.221457, 0.231886, 0.242612, 0.253639,
    0.264968, 0.276603, 0.288548, 0.300805, 0.313378, 0.326268, 0.339480, 0.353016, 0.366879, 0.381073,
    0.395599, 0.410461, 0.425662, 0.441204, 0.457091, 0.473325, 0.489909, 0.506846, 0.524138, 0.541789,
    0.559801, 0.578177, 0.596920, 0.616032, 0.635515, 0.655374, 0.675610, 0.696226, 0.717224, 0.738608,
    0.760380, 0.782542, 0.805097, 0.828048, 0.851398, 0.875148, 0.899301, 0.923861, 0.948829, 0.974208,
    1.000000,
};

esp_err_t light_set_and_update(uint8_t light_ch, uint32_t duty_a, uint32_t duty_b)
{
    uint32_t    rel_duty_a, rel_duty_b;
    uint8_t     pwm_ch_a, pwm_ch_b;
    esp_err_t   err;

    LIGHT_ARG_CHECK((light_ch > 0 )&&(light_ch <= LIGHT_MAX), "light_ch");
    LIGHT_ARG_CHECK(duty_a <= LIGHT_STD_DUTY_MAX, "duty_a");
    LIGHT_ARG_CHECK(duty_b <= LIGHT_STD_DUTY_MAX, "duty_b");
    LIGHT_ARG_CHECK(duty_a + duty_b <= LIGHT_STD_DUTY_MAX, "duty_a+b");

    rel_duty_a = gamma_correction_lut[(duty_a * LIGHT_REL_DUTY_MAX) / LIGHT_STD_DUTY_MAX] * (1 << LIGHT_DUTY_RESOLUTION);
    rel_duty_b = gamma_correction_lut[(duty_b * LIGHT_REL_DUTY_MAX) / LIGHT_STD_DUTY_MAX] * (1 << LIGHT_DUTY_RESOLUTION);
    pwm_ch_a   = light_ch * 2 - 1;
    pwm_ch_b   = light_ch * 2;

    printf("set pwm channel(%d, %d), duty a=%ld, b=%ld", pwm_ch_a, pwm_ch_b, duty_a, duty_b);
    printf("set pwm channel(%d, %d), duty a=%ld, b=%ld", pwm_ch_a, pwm_ch_b, rel_duty_a, rel_duty_b);

    err = ledc_set_duty_with_hpoint(LIGHT_PWM(pwm_ch_a).speed_mode, LIGHT_PWM(pwm_ch_a).channel, rel_duty_a, 0);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "pwm channel(%d) set duty and hpoint failed.", pwm_ch_a);
        return err;
    }
    err = ledc_set_duty_with_hpoint(LIGHT_PWM(pwm_ch_b).speed_mode, LIGHT_PWM(pwm_ch_b).channel, rel_duty_b, rel_duty_a + LIGHT_PULSE_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "pwm channel(%d) set duty and hpoint failed.", pwm_ch_b);
        return err;
    }

    err = ledc_update_duty(LIGHT_PWM(pwm_ch_a).speed_mode, LIGHT_PWM(pwm_ch_a).channel);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "pwm channel(%d) update failed.", pwm_ch_a);
        return err;
    }
    err = ledc_update_duty(LIGHT_PWM(pwm_ch_b).speed_mode, LIGHT_PWM(pwm_ch_b).channel);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "pwm channel(%d) update failed.", pwm_ch_b);
        return err;
    }

    return ESP_OK;
}