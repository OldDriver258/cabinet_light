#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_check.h"
#include "argtable3/argtable3.h"
#include "nvs.h"
#include "light.h"
/**
 * light hardware define
 */
#define LIGHT_PWM_MAX           (LIGHT_MAX * 2)
#define LIGHT_DUTY_RESOLUTION   LEDC_TIMER_12_BIT
#define LIGHT_FREQUENCY         (16000)
#define LIGHT_DUTY_MAX          ((1 << LIGHT_DUTY_RESOLUTION) - 1)
#define LIGHT_PULSE_DELAY       (4)
#define LIGHT_STD_DUTY_MAX      (100)
#define LIGHT_REL_DUTY_MAX      (95)
/**
 * light control define
 */
#define LIGHT_BRIGHTNESS_ZERO   (0)
#define LIGHT_BRIGHTNESS_MIN    (1)
#define LIGHT_BRIGHTNESS_MAX    (100)
#define LIGHT_BRIGHTNESS_ONE    (20)
#define LIGHT_BRIGHTNESS_STEP   (1)
#define LIGHT_CCT_MIN           (3000)
#define LIGHT_CCT_MAX           (6500)
#define LIGHT_CCT_STEP          ((LIGHT_CCT_MAX - LIGHT_CCT_MIN) / ((LIGHT_BRIGHTNESS_MAX - LIGHT_BRIGHTNESS_ZERO) / LIGHT_BRIGHTNESS_STEP))
#define LIGHT_UPDATE_DELAY_MS   (1)
#define LIGHT_STOP_DELAY_MS     (100)
#define LIGHT_DEFAULT_STATE     {4000, LIGHT_BRIGHTNESS_ZERO}
#define LIGHT_NVS_NAMESPACE     "light"
/**
 * light control check define
 */
#define LIGHT_CHECK(a, str, ret_val) ESP_RETURN_ON_FALSE(a, ret_val, MODULE, "%s", str)
#define LIGHT_ARG_CHECK(a, param) ESP_RETURN_ON_FALSE(a, ESP_ERR_INVALID_ARG, MODULE, param " argument is invalid")
/**
 * light control typedef
 */
typedef struct {
    bool                            ledc_initialized;               // LEDC hardware init
    bool                            module_initialized;             // Whole module init
    bool                            channel_initialized[LIGHT_MAX]; // Light channel init
} light_module_state_t;

typedef struct {
    ledc_mode_t                     speed_mode;                     // PWM timer mode
    ledc_timer_t                    timer_sel;                      // PWM timer
    ledc_channel_t                  channel;                        // PWM channel
    int                             gpio_num;                       // PWM gpio
} light_control_pwm_t;

typedef struct {
    uint32_t                        cct;                            // Light color temperature
    uint32_t                        brightness;                     // Light brightness
} light_control_state_t;

typedef struct {
    int                             channel;                        // Light channel
    const light_control_pwm_t      *cold_pwm;                       // Light cold pwm struct
    const light_control_pwm_t      *warm_pwm;                       // Light warm pwm struct
    pthread_t                       thread;                         // Light control thread
    pthread_mutex_t                 mutex;                          // Light mutex
    pthread_cond_t                  control_cond;                   // Light cond
    volatile bool                   exit_requested;                 // Light thread exit request flag
    volatile light_control_state_t  target;                         // Ligat target state
} light_control_t;

static struct {
    struct arg_int                 *channel;                        // Light channel
    struct arg_int                 *cct;                            // Light color temperature
    struct arg_int                 *brightness;                     // Light brightness
    struct arg_end                 *end;                            // Arg end
} light_control_args;
/**
 * light control module global variables
 */
static const char* MODULE = "cabinet_light";
static light_module_state_t module = {false};
/**
 * light control pwm global variables
 */
static const light_control_pwm_t pwms[LIGHT_PWM_MAX] = {
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
};
/**
 * light control global variables
 */
static light_control_t lights[LIGHT_MAX];
/**
 * light control gamma correction global variables
 * 
 * Brightness 0 - 100% gamma correction look up table (gamma = 2.6)
 * Y = B ^ 2.6
 * Pre-computed LUT to save some runtime computation
 */
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
/**
 * @brief Initialize light control lec hardware
 * @return Error code
 */
static esp_err_t light_control_ledc_init(void)
{
    ledc_timer_config_t     ledc_timer;
    ledc_channel_config_t   ledc_channel;
    esp_err_t               err;
    int                     pwm_ch;

    if (module.ledc_initialized) {
        return ESP_OK;
    }

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
        ledc_channel.speed_mode = pwms[pwm_ch].speed_mode;
        ledc_channel.channel    = pwms[pwm_ch].channel;
        ledc_channel.timer_sel  = pwms[pwm_ch].timer_sel;
        ledc_channel.gpio_num   = pwms[pwm_ch].gpio_num;
        err = ledc_channel_config(&ledc_channel);
        if (err != ESP_OK) {
            ESP_LOGE(MODULE, "pwm channel(%d) init failed.", pwm_ch);
            return err;
        }
    }

    if (err == ESP_OK) {
        module.ledc_initialized = true;
    }

    return ESP_OK;
}
/**
 * @brief Calculate the proportion of warm and cold light by color temperature
 * @param state Light state
 * @param cold_factor The proportion of cold light
 * @param warm_factor The proportion of warm light
 */
static void light_control_calc_factor (light_control_state_t *state, float *cold_factor, float *warm_factor)
{
    *cold_factor = (float)(state->cct - LIGHT_CCT_MIN) / (LIGHT_CCT_MAX - LIGHT_CCT_MIN);
    *warm_factor = (float)1 - *cold_factor;
}
/**
 * @brief Calculate the corresponding gamma parameter by brightness
 * @param state Light state
 * @return Gamma parameter
 */
static float light_control_calc_gamma(light_control_state_t *state)
{
    uint32_t brightness;

    if (state->brightness != LIGHT_BRIGHTNESS_ZERO) {
        brightness = ((state->brightness - LIGHT_BRIGHTNESS_MIN) * 
                      (LIGHT_BRIGHTNESS_MAX - LIGHT_BRIGHTNESS_ONE) / 
                      (LIGHT_BRIGHTNESS_MAX - LIGHT_BRIGHTNESS_MIN)) + 
                     LIGHT_BRIGHTNESS_ONE;
    } else {
        brightness = state->brightness;
    }

    return gamma_correction_lut[brightness];
}
/**
 * @brief Adjust hardware duty to protect the hardware, the maximum duty cycle does not exceed 95%.
 * @param duty Duty before adjustment
 * @return Duty after adjustment
 */
static uint32_t light_control_hw_duty_adjustment(uint32_t duty)
{
    return duty * LIGHT_REL_DUTY_MAX / LIGHT_STD_DUTY_MAX;
}
/**
 * @brief Set the corresponding duty cycle to the hardware
 * @param light Light struct
 * @param rel_cold_duty Real cold duty cycle
 * @param rel_warm_duty Real warm duty cycle
 * @return 
 */
static esp_err_t light_control_set_and_update(light_control_t *light, uint32_t rel_cold_duty, uint32_t rel_warm_duty)
{
    esp_err_t           err;

    err = ledc_set_duty_with_hpoint(light->cold_pwm->speed_mode, light->cold_pwm->channel, 
                                    rel_cold_duty, 0);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Ledc %s speed timer channel(%d) set duty and hpoint failed.", 
                 (light->cold_pwm->speed_mode == LEDC_HIGH_SPEED_MODE) ? "high" : "low", 
                 light->cold_pwm->channel);
        return err;
    }
    err = ledc_set_duty_with_hpoint(light->warm_pwm->speed_mode, light->warm_pwm->channel, 
                                    rel_warm_duty, rel_cold_duty + LIGHT_PULSE_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Ledc %s speed timer channel(%d) set duty and hpoint failed.", 
                 (light->warm_pwm->speed_mode == LEDC_HIGH_SPEED_MODE) ? "high" : "low", 
                 light->warm_pwm->channel);
        return err;
    }

    err = ledc_update_duty(light->cold_pwm->speed_mode, light->cold_pwm->channel);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Ledc %s speed timer channel(%d) update failed.", 
            (light->cold_pwm->speed_mode == LEDC_HIGH_SPEED_MODE) ? "high" : "low", 
            light->cold_pwm->channel);
        return err;
    }
    err = ledc_update_duty(light->warm_pwm->speed_mode, light->warm_pwm->channel);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Ledc %s speed timer channel(%d) update failed.", 
            (light->warm_pwm->speed_mode == LEDC_HIGH_SPEED_MODE) ? "high" : "low", 
            light->warm_pwm->channel);
        return err;
    }

    return ESP_OK;
}
/**
 * @brief Sets the light to the specified state
 * @param light Light struct
 * @param state Light state
 * @return Error code
 */
static esp_err_t light_control_set_state(light_control_t *light, light_control_state_t *state)
{
    float       cold_factor, warm_factor;
    float       total_gamma;
    uint32_t    rel_cold_duty, rel_warm_duty;

    LIGHT_ARG_CHECK((state->cct >= LIGHT_CCT_MIN) && (state->cct <= LIGHT_CCT_MAX), "state->cct");
    LIGHT_ARG_CHECK(state->brightness <= LIGHT_BRIGHTNESS_MAX, "state->brightness");

    // Calculate cold and warm light factor
    light_control_calc_factor(state, &cold_factor, &warm_factor);

    // Calculate gamma corresponding to brightness
    total_gamma = light_control_calc_gamma(state);

    // Calculate harmware duty
    rel_cold_duty = total_gamma * cold_factor * (1 << LIGHT_DUTY_RESOLUTION);
    rel_warm_duty = total_gamma * warm_factor * (1 << LIGHT_DUTY_RESOLUTION);

    // Adjust the max duty <= 95%
    rel_cold_duty = light_control_hw_duty_adjustment(rel_cold_duty);
    rel_warm_duty = light_control_hw_duty_adjustment(rel_warm_duty);

    // Update to hardware
    return light_control_set_and_update(light, rel_cold_duty, rel_warm_duty);
}
/**
 * @brief Light control delay
 * @param light Light struct
 * @param ms Millisecond
 */
static void light_control_timewait(light_control_t *light, uint32_t ms)
{
    struct timespec timeout;

    pthread_mutex_lock(&light->mutex);

    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_nsec += ms * 1000 * 1000;
    if (timeout.tv_nsec >= 1000000000) {
        timeout.tv_sec += 1;
        timeout.tv_nsec -= 1000000000;
    }
    pthread_cond_timedwait(&light->control_cond, &light->mutex, &timeout);

    pthread_mutex_unlock(&light->mutex);
}
/**
 * @brief Set the target state of the light
 * @param light Light struct
 * @param state Target state
 */
static void light_control_set_target(light_control_t *light, light_control_state_t *state)
{
    pthread_mutex_lock(&light->mutex);

    light->target.brightness = state->brightness;
    light->target.cct        = state->cct;
    pthread_cond_signal(&light->control_cond);

    pthread_mutex_unlock(&light->mutex);
}
/**
 * @brief Storage light status
 * @param light Light struct
 * @param state Store state
 * @return Error code
 */
static esp_err_t light_control_storage_set(light_control_t *light, light_control_state_t *state)
{
    char            key[NVS_KEY_NAME_MAX_SIZE];
    esp_err_t       err;
    nvs_handle_t    handle;

    err = nvs_open(LIGHT_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return err;
    }

    snprintf(key, sizeof(key), "light%d_cct", light->channel);
    err = nvs_set_u32(handle, key, state->cct);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Error (%s) set NVS u32 cct!\n", esp_err_to_name(err));
    }

    snprintf(key, sizeof(key), "light%d_brt", light->channel);
    err = nvs_set_u32(handle, key, state->brightness);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Error (%s) set NVS u32 brightness!\n", esp_err_to_name(err));
    }

    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Error (%s) NVS commit failed!\n", esp_err_to_name(err));
    }

    nvs_close(handle);

    return err;
}
/**
 * @brief Read the status of the light
 * @param light Light struct
 * @param state Read state
 * @return Error code
 */
static esp_err_t light_control_storage_get(light_control_t *light, light_control_state_t *state)
{
    char            key[NVS_KEY_NAME_MAX_SIZE];
    esp_err_t       err;
    nvs_handle_t    handle;

    err = nvs_open(LIGHT_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return err;
    }

    snprintf(key, sizeof(key), "light%d_cct", light->channel);
    err = nvs_get_u32(handle, key, &state->cct);
    switch (err) {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGE(MODULE, "The value is not initialized yet!\n");
            break;
        default :
            ESP_LOGE(MODULE, "Error (%s) reading!\n", esp_err_to_name(err));
    }

    snprintf(key, sizeof(key), "light%d_brt", light->channel);
    err = nvs_get_u32(handle, key, &state->brightness);
    switch (err) {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGE(MODULE, "The value is not initialized yet!\n");
            break;
        default :
            ESP_LOGE(MODULE, "Error (%s) reading!\n", esp_err_to_name(err));
    }

    nvs_close(handle);

    return err;
}
/**
 * @brief Light control command line program
 * @param argc 
 * @param argv 
 * @return Error code
 */
static int light_control_cmd(int argc, char **argv)
{
    int                     nerrors;
    uint8_t                 light_ch;
    light_control_state_t   state;
    
    nerrors = arg_parse(argc, argv, (void **) &light_control_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, light_control_args.end, argv[0]);
        return 1;
    }

    // Get light status from command line
    light_ch            = light_control_args.channel->ival[0] - 1;
    state.cct           = light_control_args.cct->ival[0];
    state.brightness    = light_control_args.brightness->ival[0];

    // Check light state
    if (!module.channel_initialized[light_ch]) {
        ESP_LOGE(MODULE, "Select channel %d did not initialized! Start it first", light_ch + 1);
        return ESP_ERR_INVALID_STATE;
    }

    // Check correct light param
    LIGHT_ARG_CHECK(light_ch < LIGHT_MAX, "light_ch");
    LIGHT_ARG_CHECK((state.cct >= LIGHT_CCT_MIN) && (state.cct <= LIGHT_CCT_MAX), "cct");
    LIGHT_ARG_CHECK(state.brightness <= LIGHT_BRIGHTNESS_MAX, "brightness");

    // Set light target
    light_control_set_target(&lights[light_ch], &state);

    return ESP_OK;
}
/**
 * @brief Register the light control command line program
 * @return Error code
 */
static esp_err_t light_control_register_cmd(void)
{
    static char channel_range[32], temperature_range[32], brightness_range[32];

    snprintf(channel_range, sizeof(channel_range), "<%d-%d>", 1, LIGHT_MAX);
    snprintf(brightness_range, sizeof(brightness_range), "<%d-%d>", LIGHT_BRIGHTNESS_ZERO, LIGHT_BRIGHTNESS_MAX);
    snprintf(temperature_range, sizeof(temperature_range), "<%d-%d>", LIGHT_CCT_MIN, LIGHT_CCT_MAX);

    light_control_args.channel = 
        arg_int1("c", "channel", channel_range, "Select a light channel to control.");
    light_control_args.cct = 
        arg_int1("t", "temperature", temperature_range, "Set target color temperature.");
    light_control_args.brightness = 
        arg_int1("b", "brightness", brightness_range, "Set target brightness.");
    light_control_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command    = "light",
        .help       = "Control light status."
                      "Specify color temperature and brightness.",
        .hint       = NULL,
        .func       = &light_control_cmd,
        .argtable   = &light_control_args
    };

    return esp_console_cmd_register(&cmd);
}
/**
 * @brief Light control thread
 * @param arg Light struct
 * @return Exit code
 */
static void *light_control_thread(void *arg)
{
    light_control_t        *light       = (light_control_t*)arg;
    light_control_state_t   start_state = LIGHT_DEFAULT_STATE;
    light_control_state_t   current, target;

    current.brightness = LIGHT_BRIGHTNESS_ZERO;
    current.cct        = (LIGHT_CCT_MAX + LIGHT_CCT_MIN) / 2;

    light_control_storage_get(light, &start_state);
    light_control_set_target(light, &start_state);

    while (true)
    {
        // Check if you should exit
        pthread_mutex_lock(&light->mutex);

        if (light->exit_requested) {
            pthread_mutex_unlock(&light->mutex);
            break;
        }

        // Read target status
        target = light->target;
        pthread_mutex_unlock(&light->mutex);

        if ((current.brightness != target.brightness) ||
            (current.cct        != target.cct)) {
            // Smooth brightness adjustment
            if (target.brightness > current.brightness) {
                current.brightness += LIGHT_BRIGHTNESS_STEP;
                if (current.brightness > target.brightness) {
                    current.brightness = target.brightness;
                }
            } else if (target.brightness < current.brightness) {
                current.brightness -= LIGHT_BRIGHTNESS_STEP;
                if (current.brightness < target.brightness) {
                    current.brightness = target.brightness;
                }
            }

            // Smoothly adjust color temperature
            if (target.cct > current.cct) {
                current.cct += LIGHT_CCT_STEP;
                if (current.cct > target.cct) {
                    current.cct = target.cct;
                }
            } else if (target.cct < current.cct) {
                current.cct -= LIGHT_CCT_STEP;
                if (current.cct < target.cct) {
                    current.cct = target.cct;
                }
            }
            
            // Update color temperature and brightness
            light_control_set_state(light, &current);

            if ((current.brightness == target.brightness) ||
                (current.cct        == target.cct)) {
                    light_control_storage_set(light, &current);
                }

            light_control_timewait(light, LIGHT_UPDATE_DELAY_MS);
        } else {
            // Idle wait, just check for exit
            light_control_timewait(light, LIGHT_STOP_DELAY_MS);
        }
    }
    
    return NULL;
}
/**
 * @brief Start the light control of the corresponding channel
 * @param channel Light channel
 * @return Error code
 */
esp_err_t light_control_start(int channel)
{
    light_control_t    *light;
    uint8_t             light_ch;
    pthread_attr_t      attr;
    esp_err_t           err;

    LIGHT_ARG_CHECK((channel > 0) && (channel <= LIGHT_MAX), "channel");

    if (!module.ledc_initialized) {
        ESP_LOGE(MODULE, "LEDC hardware not initialized, call light_module_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    light_ch        = channel - 1;
    light           = &lights[light_ch];

    if (module.channel_initialized[light_ch]) {
        ESP_LOGW(MODULE, "Channel %d already initialized, stopping first", channel);
        light_control_stop(channel);
    }

    memset(light, 0, sizeof(light_control_t));
    light->channel  = channel;
    light->warm_pwm = &pwms[light_ch * 2];
    light->cold_pwm = &pwms[light_ch * 2 + 1];

    err = pthread_attr_init(&attr);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Failed init pthread attr.");
        return err;
    }

    err = pthread_mutex_init(&light->mutex, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Failed init mutex(%d).", light_ch);

        pthread_attr_destroy(&attr);
        return err;
    }

    err = pthread_cond_init(&light->control_cond, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Failed init condition variable(%d).", light_ch);

        pthread_mutex_destroy(&light->mutex);
        pthread_attr_destroy(&attr);
        return err;
    }

    // start light control
    pthread_mutex_lock(&light->mutex);
    light->exit_requested = false;
    pthread_mutex_unlock(&light->mutex);

    err = pthread_create(&light->thread, &attr, light_control_thread, light);
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Failed creat thread(%d).", light_ch);

        pthread_cond_destroy(&light->control_cond);
        pthread_mutex_destroy(&light->mutex);
        pthread_attr_destroy(&attr);
        return err;
    }

    pthread_attr_destroy(&attr);

    module.channel_initialized[light_ch] = true;

    return ESP_OK;
}
/**
 * @brief Stop the light control of the corresponding channel
 * @param channel Light channel
 * @return Error code
 */
esp_err_t light_control_stop(int channel)
{
    light_control_t    *light;
    uint8_t             light_ch;

    LIGHT_ARG_CHECK((channel > 0) && (channel <= LIGHT_MAX), "channel");

    light_ch    = channel - 1;
    light       = &lights[light_ch];

    if (light->thread) {
        // stop light control
        pthread_mutex_lock(&light->mutex);
        light->exit_requested  = true;
        pthread_cond_signal(&light->control_cond);
        pthread_mutex_unlock(&light->mutex);

        pthread_join(light->thread, NULL);
        light->thread = 0;
    }

    if (light->control_cond) {
        pthread_cond_destroy(&light->control_cond);
    }

    if (light->mutex) {
        pthread_mutex_destroy(&light->mutex);
    }

    module.channel_initialized[light_ch] = false;

    return ESP_OK;
}
/**
 * @brief Light module initialization
 * @return Error code
 */
esp_err_t light_module_init(void)
{
    esp_err_t err;

    if (module.module_initialized) {
        return ESP_OK;
    }

    err = light_control_ledc_init();
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Failed to initialize LEDC hardware");
        return err;
    }

    err = light_control_register_cmd();
    if (err != ESP_OK) {
        ESP_LOGE(MODULE, "Failed to register console commands");
        return err;
    }

    module.module_initialized = true;
    
    return ESP_OK;
}

