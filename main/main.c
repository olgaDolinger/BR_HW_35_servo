#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"


static const char *TAG = "SERVO";

/* ---------- Піни ---------- */
#define SERVO_GPIO          4

/* ---------- PWM (LEDC) ---------- */
#define SERVO_TIMER         LEDC_TIMER_0
#define SERVO_CHANNEL       LEDC_CHANNEL_0
#define SERVO_MODE          LEDC_LOW_SPEED_MODE
#define SERVO_FREQ_HZ       50                 // період 20 мс
#define SERVO_RES           LEDC_TIMER_14_BIT
#define SERVO_DUTY_MAX      ((1 << 14) - 1)    // 16383
#define SERVO_PERIOD_US     20000

/* ---------- Параметри SG90 ---------- */
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2400
#define SERVO_MAX_ANGLE     180                // кут між крайніми положеннями

/* ---------- Межі duty (рахуються з імпульсів) ---------- */
#define DUTY_MIN            ((SERVO_MIN_US * SERVO_DUTY_MAX) / SERVO_PERIOD_US)   // ≈ 409
#define DUTY_MAX            ((SERVO_MAX_US * SERVO_DUTY_MAX) / SERVO_PERIOD_US)   // ≈ 1965

/* ---------- Потенціометр ---------- */
#define POT_GPIO            1                  // аналоговий вхід для потенціометра
#define POT_CHANNEL         ADC_CHANNEL_0      // POT_GPIO = 1 -> ADC_CHANNEL_0
#define POT_READ_DELAY_MS   20                 // період опитування
#define POT_MAX_ANGLE       270                // повний хід ручки
#define POT_RAW_MIN         0                  // raw у лівому упорі (підібрати по логу)
#define POT_RAW_MAX         4095               // raw у правому упорі (підібрати по логу)

static adc_oneshot_unit_handle_t adc_handle;

/* ---------- Ініціалізація ADC для потенціометра ---------- */
static void pot_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT_CHANNEL, &chan_config));
}



/* ---------- Ініціалізація PWM для сервоприводу ---------- */
static void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = SERVO_MODE,
        .timer_num       = SERVO_TIMER,
        .duty_resolution = SERVO_RES,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel    = SERVO_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .duty       = DUTY_MIN,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

/* ---------- Зчитування значення потенціометра ---------- */
static int pot_read(void)
{
    int raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, POT_CHANNEL, &raw));
    return raw;
}

/* ---------- Встановлення ШІМ (duty cycle) для сервоприводу ---------- */
static void servo_set_duty(int duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, (uint32_t)duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_MODE, SERVO_CHANNEL));
}

/* кут ручки від лівого упору: 0..POT_MAX_ANGLE */
static int pot_to_angle(int pot_val)
{
    if (pot_val < POT_RAW_MIN) pot_val = POT_RAW_MIN;
    if (pot_val > POT_RAW_MAX) pot_val = POT_RAW_MAX;
    return ((pot_val - POT_RAW_MIN) * POT_MAX_ANGLE) / (POT_RAW_MAX - POT_RAW_MIN);
}

/* 1:1 — частина ходу ручки понад SERVO_MAX_ANGLE обрізається */
static int clamp_servo_angle(int angle)
{
    return angle > SERVO_MAX_ANGLE ? SERVO_MAX_ANGLE : angle;
}

static int angle_to_duty(int angle)
{
    return DUTY_MIN + (angle * (DUTY_MAX - DUTY_MIN)) / SERVO_MAX_ANGLE;
}

void app_main(void)
{
    pot_init();
    servo_init();
    ESP_LOGI(TAG, "Старт. Серво керується потенціометром 1:1 (GPIO%d)", POT_GPIO);

    int last_angle = -1;

    while (1) {
        int pot_val     = pot_read();
        int pot_angle   = pot_to_angle(pot_val);
        int servo_angle = clamp_servo_angle(pot_angle);
        int duty        = angle_to_duty(servo_angle);

        servo_set_duty(duty);

        if (pot_angle != last_angle) {     // логуємо лише при зміні кута ручки
            if (pot_angle > SERVO_MAX_ANGLE) {
                ESP_LOGW(TAG, "кут від лівого краю: %d° — межа сервоприводу, ручка поза діапазоном (%d°)",
                         servo_angle, pot_angle);
            } else {
                ESP_LOGI(TAG, "кут від лівого краю: %d° (pot: %d, duty: %d)",
                         servo_angle, pot_val, duty);
            }
            last_angle = pot_angle;
        }

        vTaskDelay(pdMS_TO_TICKS(POT_READ_DELAY_MS));
    }
}