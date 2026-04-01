#include "esp-sparkmax.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/queue.h"

static const char *TAG = "sparkmax";

typedef struct {
    uint32_t arb_id;
    uint8_t  data[8];
    uint8_t  len;
} raw_frame_t;

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static void send_raw(sparkmax_t *motor,
                     uint32_t arb_id, const uint8_t *data, uint8_t len)
{
    if (len > 8) len = 8;
    esp_err_t err = motor->config.tx_fn(arb_id, data, len, motor->config.tx_ctx);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TX failed arb_id=0x%08X err=%d", (unsigned)arb_id, err);
    }
}

static void send_setpoint(sparkmax_t *motor, uint32_t arb_id, float value)
{
    uint8_t data[8] = {0};
    memcpy(data, &value, sizeof(float));
    send_raw(motor, arb_id, data, sizeof(data));
}

// ---------------------------------------------------------------------------
// Heartbeat task — 8×0xFF every 10 ms
// ---------------------------------------------------------------------------

static void heartbeat_task(void *arg)
{
    sparkmax_t *motor = (sparkmax_t *)arg;
    static const uint8_t hb_data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    };
    while (1) {
        send_raw(motor, SPARKMAX_HEARTBEAT_ARBID, hb_data, sizeof(hb_data));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------------------------------------------------------------------------
// Decode status frames (task context — FPU safe)
// ---------------------------------------------------------------------------

static void decode_status(sparkmax_t *motor,
                           uint32_t arb_id, const uint8_t *data, uint8_t len)
{
    uint32_t base = arb_id - motor->device_id;

    if (base == SPARKMAX_STATUS_0_BASE && len >= 2) {
        int16_t raw;
        memcpy(&raw, data, sizeof(raw));
        motor->applied_output = (float)raw / 32768.0f;

    } else if (base == SPARKMAX_STATUS_1_BASE && len >= 8) {
        float vel;
        memcpy(&vel, data, sizeof(vel));
        motor->velocity_rpm = vel;

        motor->temperature_c = (float)data[4];

        uint16_t v_raw = (uint16_t)data[5] | (((uint16_t)data[6] & 0xF0) << 4);
        motor->bus_voltage = (float)v_raw / 128.0f;

        uint16_t i_raw = ((uint16_t)data[6] & 0x0F) | ((uint16_t)data[7] << 4);
        motor->output_current = (float)i_raw / 32.0f;

    } else if (base == SPARKMAX_STATUS_2_BASE && len >= 4) {
        float pos;
        memcpy(&pos, data, sizeof(pos));
        motor->position_rotations = pos;
    }
}

// ---------------------------------------------------------------------------
// Decode task
// ---------------------------------------------------------------------------

static void decode_task(void *arg)
{
    sparkmax_t *motor = (sparkmax_t *)arg;
    raw_frame_t f;
    while (1) {
        if (xQueueReceive(motor->rx_queue, &f, portMAX_DELAY) == pdTRUE) {
            decode_status(motor, f.arb_id, f.data, f.len);
            if (motor->rx_cb) {
                motor->rx_cb(f.arb_id, f.data, f.len, motor->rx_user_ctx);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void sparkmax_init(sparkmax_t *motor,
                   const sparkmax_config_t *config,
                   uint8_t device_id)
{
    memset(motor, 0, sizeof(*motor));
    motor->config    = *config;
    motor->device_id = device_id;
}

bool sparkmax_begin(sparkmax_t *motor,
                    sparkmax_rx_cb_t rx_cb,
                    void *user_ctx)
{
    if (!motor->config.tx_fn) {
        ESP_LOGE(TAG, "tx_fn is NULL — provide a transmit function in sparkmax_config_t");
        return false;
    }

    motor->rx_cb       = rx_cb;
    motor->rx_user_ctx = user_ctx;

    motor->rx_queue = xQueueCreate(16, sizeof(raw_frame_t));
    if (!motor->rx_queue) {
        ESP_LOGE(TAG, "Failed to create rx queue");
        return false;
    }

    BaseType_t ret = xTaskCreate(heartbeat_task, "sparkmax_hb", 2048, motor,
                                 configMAX_PRIORITIES - 2, &motor->hb_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create heartbeat task");
        vQueueDelete(motor->rx_queue);
        motor->rx_queue = NULL;
        return false;
    }

    ret = xTaskCreate(decode_task, "sparkmax_rx", 2048, motor,
                      configMAX_PRIORITIES - 3, &motor->decode_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create decode task");
        vTaskDelete(motor->hb_task);
        motor->hb_task = NULL;
        vQueueDelete(motor->rx_queue);
        motor->rx_queue = NULL;
        return false;
    }

    motor->started = true;
    ESP_LOGI(TAG, "SPARK MAX started (device_id=%d)", motor->device_id);
    return true;
}

void sparkmax_end(sparkmax_t *motor)
{
    if (!motor->started) return;

    if (motor->hb_task) {
        vTaskDelete(motor->hb_task);
        motor->hb_task = NULL;
    }
    if (motor->decode_task) {
        vTaskDelete(motor->decode_task);
        motor->decode_task = NULL;
    }

    sparkmax_set_duty_cycle(motor, 0.0f);

    if (motor->rx_queue) {
        vQueueDelete(motor->rx_queue);
        motor->rx_queue = NULL;
    }

    motor->started = false;
}

void sparkmax_feed_rx(sparkmax_t *motor,
                      uint32_t arb_id,
                      const uint8_t *data,
                      uint8_t len)
{
    if (!motor->rx_queue) return;
    if (len > 8) len = 8;

    raw_frame_t f;
    f.arb_id = arb_id;
    f.len    = len;
    memcpy(f.data, data, len);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(motor->rx_queue, &f, &woken);
    portYIELD_FROM_ISR(woken);
}

void sparkmax_set_duty_cycle(sparkmax_t *motor, float duty)
{
    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;
    send_setpoint(motor, SPARKMAX_ARBID(SPARKMAX_DUTY_CYCLE_BASE, motor->device_id), duty);
}

void sparkmax_set_velocity(sparkmax_t *motor, float rpm)
{
    send_setpoint(motor, SPARKMAX_ARBID(SPARKMAX_VELOCITY_BASE, motor->device_id), rpm);
}

void sparkmax_set_position(sparkmax_t *motor, float rotations)
{
    send_setpoint(motor, SPARKMAX_ARBID(SPARKMAX_POSITION_BASE, motor->device_id), rotations);
}

void sparkmax_set_status_period(sparkmax_t *motor,
                                uint32_t frame_base, uint16_t period_ms)
{
    uint8_t data[2] = { (uint8_t)(period_ms & 0xFF), (uint8_t)(period_ms >> 8) };
    send_raw(motor, SPARKMAX_ARBID(frame_base, motor->device_id), data, sizeof(data));
}

float sparkmax_get_position(sparkmax_t *motor)    { return motor->position_rotations; }
float sparkmax_get_velocity(sparkmax_t *motor)    { return motor->velocity_rpm; }
float sparkmax_get_voltage(sparkmax_t *motor)     { return motor->bus_voltage; }
float sparkmax_get_current(sparkmax_t *motor)     { return motor->output_current; }
float sparkmax_get_output(sparkmax_t *motor)      { return motor->applied_output; }
float sparkmax_get_temperature(sparkmax_t *motor) { return motor->temperature_c; }

