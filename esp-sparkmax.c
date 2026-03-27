#include "esp-sparkmax.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/queue.h"

static const char *TAG = "sparkmax";

/* Raw frame passed from RX ISR to decode task — no floats in ISR */
typedef struct {
    uint32_t arb_id;
    uint8_t  data[8];
    uint8_t  len;
} raw_frame_t;

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static void send_frame(sparkmax_t *motor,
                       twai_frame_t *frame, uint8_t *buf,
                       uint32_t arb_id, const uint8_t *data, uint8_t len)
{
    if (!motor->node) return;
    if (len > 8) len = 8;

    memcpy(buf, data, len);

    frame->header.id  = arb_id;
    frame->header.ide = true;
    frame->header.rtr = false;
    frame->header.dlc = len;
    frame->header.fdf = false;
    frame->header.brs = false;
    frame->header.esi = false;
    frame->buffer     = buf;
    frame->buffer_len = len;

    esp_err_t err = twai_node_transmit(motor->node, frame, 5);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TX failed arb_id=0x%08X err=%d", (unsigned)arb_id, err);
    }
}

static void send_setpoint(sparkmax_t *motor, uint32_t arb_id, float value)
{
    uint8_t data[8] = {0};
    memcpy(data, &value, sizeof(float));
    send_frame(motor, &motor->tx_frame_cmd, motor->tx_buf_cmd,
               arb_id, data, sizeof(data));
}

// ---------------------------------------------------------------------------
// Heartbeat task — 8×0xFF broadcast every 10 ms
// ---------------------------------------------------------------------------

static void heartbeat_task(void *arg)
{
    sparkmax_t *motor = (sparkmax_t *)arg;
    static const uint8_t hb_data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    };

    while (1) {
        send_frame(motor, &motor->tx_frame_hb, motor->tx_buf_hb,
                   SPARKMAX_HEARTBEAT_ARBID, hb_data, sizeof(hb_data));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------------------------------------------------------------------------
// Decode periodic status frames  (runs in task context — FPU is safe here)
// ---------------------------------------------------------------------------

static void decode_status(sparkmax_t *motor, uint32_t arb_id, const uint8_t *data, uint8_t len) {
    uint32_t base = arb_id - motor->device_id;   /* strip device ID */

    if (base == SPARKMAX_STATUS_0_BASE && len >= 2) {
        /* Applied output: signed 16-bit, scale 1/32768 */
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
        /* Position: IEEE float in bytes 0-3 (rotations) */
        float pos;
        memcpy(&pos, data, sizeof(pos));
        motor->position_rotations = pos;
    }
}

// ---------------------------------------------------------------------------
// Decode task — dequeues raw frames and decodes them (FPU safe)
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
// RX ISR  — only copies bytes into the queue, no float math
// ---------------------------------------------------------------------------

static bool rx_isr(twai_node_handle_t handle,
                   const twai_rx_done_event_data_t *edata,
                   void *user_ctx)
{
    sparkmax_t *motor = (sparkmax_t *)user_ctx;
    raw_frame_t f;
    memset(&f, 0, sizeof(f));
    twai_frame_t frame = { .buffer = f.data, .buffer_len = sizeof(f.data) };

    if (twai_node_receive_from_isr(handle, &frame) == ESP_OK) {
        f.arb_id = frame.header.id;
        f.len    = (uint8_t)frame.header.dlc;
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(motor->rx_queue, &f, &woken);
        return woken == pdTRUE;
    }
    return false;
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
    motor->rx_cb       = rx_cb;
    motor->rx_user_ctx = user_ctx;

    motor->rx_queue = xQueueCreate(16, sizeof(raw_frame_t));
    if (!motor->rx_queue) {
        ESP_LOGE(TAG, "Failed to create rx queue");
        return false;
    }

    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx                = motor->config.tx_pin,
            .rx                = motor->config.rx_pin,
            .quanta_clk_out    = -1,
            .bus_off_indicator = -1,
        },
        .bit_timing     = { .bitrate = 1000000 },  /* 1 Mbit/s */
        .tx_queue_depth = 16,
        .flags = { 0 },
    };

    esp_err_t err = twai_new_node_onchip(&node_cfg, &motor->node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_new_node_onchip failed: %d", err);
        vQueueDelete(motor->rx_queue);
        motor->rx_queue = NULL;
        return false;
    }

    twai_event_callbacks_t cbs = { .on_rx_done = rx_isr };
    err = twai_node_register_event_callbacks(motor->node, &cbs, motor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register_event_callbacks failed: %d", err);
        twai_node_delete(motor->node);
        motor->node = NULL;
        vQueueDelete(motor->rx_queue);
        motor->rx_queue = NULL;
        return false;
    }

    err = twai_node_enable(motor->node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_enable failed: %d", err);
        twai_node_delete(motor->node);
        motor->node = NULL;
        vQueueDelete(motor->rx_queue);
        motor->rx_queue = NULL;
        return false;
    }

    BaseType_t ret = xTaskCreate(heartbeat_task, "sparkmax_hb", 2048, motor,
                                 configMAX_PRIORITIES - 2, &motor->hb_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create heartbeat task");
        twai_node_disable(motor->node);
        twai_node_delete(motor->node);
        motor->node = NULL;
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
        twai_node_disable(motor->node);
        twai_node_delete(motor->node);
        motor->node = NULL;
        vQueueDelete(motor->rx_queue);
        motor->rx_queue = NULL;
        return false;
    }

    motor->started = true;
    ESP_LOGI(TAG, "SPARK MAX started (device_id=%d)", motor->device_id);
    ESP_LOGI(TAG, "  heartbeat  0x%08X (broadcast)", (unsigned)SPARKMAX_HEARTBEAT_ARBID);
    ESP_LOGI(TAG, "  duty cycle 0x%08X", (unsigned)SPARKMAX_ARBID(SPARKMAX_DUTY_CYCLE_BASE, motor->device_id));
    ESP_LOGI(TAG, "  velocity   0x%08X", (unsigned)SPARKMAX_ARBID(SPARKMAX_VELOCITY_BASE,   motor->device_id));
    ESP_LOGI(TAG, "  position   0x%08X", (unsigned)SPARKMAX_ARBID(SPARKMAX_POSITION_BASE,   motor->device_id));
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

    twai_node_disable(motor->node);
    twai_node_delete(motor->node);
    motor->node    = NULL;

    if (motor->rx_queue) {
        vQueueDelete(motor->rx_queue);
        motor->rx_queue = NULL;
    }

    motor->started = false;
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
    send_frame(motor, &motor->tx_frame_status, motor->tx_buf_status,
               SPARKMAX_ARBID(frame_base, motor->device_id), data, sizeof(data));
}

// ---------------------------------------------------------------------------
// Status getters
// ---------------------------------------------------------------------------

float sparkmax_get_position(sparkmax_t *motor)   { return motor->position_rotations; }
float sparkmax_get_velocity(sparkmax_t *motor)   { return motor->velocity_rpm; }
float sparkmax_get_voltage(sparkmax_t *motor)    { return motor->bus_voltage; }
float sparkmax_get_current(sparkmax_t *motor)    { return motor->output_current; }
float sparkmax_get_output(sparkmax_t *motor)     { return motor->applied_output; }
float sparkmax_get_temperature(sparkmax_t *motor) { return motor->temperature_c; }

