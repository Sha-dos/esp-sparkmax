#include "esp-sparkmax.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "sparkmax";

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/*
 * Populate a long-lived twai_frame_t and queue it for transmission.
 *
 * Both `frame` and `buf` must be members of sparkmax_t — the TWAI driver
 * stores a raw pointer to the frame and the ISR dereferences it after this
 * function returns.  Stack-allocated frames will be freed before that happens.
 */
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

    /* API takes milliseconds directly, not FreeRTOS ticks */
    esp_err_t err = twai_node_transmit(motor->node, frame, 5);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TX failed arb_id=0x%08X err=%d", (unsigned)arb_id, err);
    }
}

/* Convenience: pack a float setpoint into an 8-byte frame and send it. */
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
// RX ISR
// ---------------------------------------------------------------------------

static bool rx_isr(twai_node_handle_t handle,
                   const twai_rx_done_event_data_t *edata,
                   void *user_ctx)
{
    sparkmax_t *motor = (sparkmax_t *)user_ctx;
    uint8_t buf[8] = {0};
    twai_frame_t frame = { .buffer = buf, .buffer_len = sizeof(buf) };

    if (twai_node_receive_from_isr(handle, &frame) == ESP_OK && motor->rx_cb) {
        motor->rx_cb(frame.header.id, buf, (uint8_t)frame.buffer_len,
                     motor->rx_user_ctx);
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

    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx                = motor->config.tx_pin,
            .rx                = motor->config.rx_pin,
            .quanta_clk_out    = -1,
            .bus_off_indicator = -1,
        },
        .bit_timing     = { .bitrate = 1000000 },  /* 1 Mbit/s */
        .tx_queue_depth = 16,
        .flags = {
            /* STM bit: transmit succeeds without an external ACK.
               Without this the ESP32 bus-errors when no other node
               is present to acknowledge frames. */
            .enable_self_test = 1,
        },
    };

    esp_err_t err = twai_new_node_onchip(&node_cfg, &motor->node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_new_node_onchip failed: %d", err);
        return false;
    }

    if (rx_cb) {
        twai_event_callbacks_t cbs = { .on_rx_done = rx_isr };
        err = twai_node_register_event_callbacks(motor->node, &cbs, motor);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "register_event_callbacks failed: %d", err);
            twai_node_delete(motor->node);
            motor->node = NULL;
            return false;
        }
    }

    err = twai_node_enable(motor->node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_enable failed: %d", err);
        twai_node_delete(motor->node);
        motor->node = NULL;
        return false;
    }

    BaseType_t ret = xTaskCreate(heartbeat_task, "sparkmax_hb", 2048, motor,
                                 configMAX_PRIORITIES - 2, &motor->hb_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create heartbeat task");
        twai_node_disable(motor->node);
        twai_node_delete(motor->node);
        motor->node = NULL;
        return false;
    }

    motor->started = true;
    ESP_LOGI(TAG, "SPARK MAX started  device_id=%d", motor->device_id);
    ESP_LOGI(TAG, "  heartbeat  0x%08X (broadcast)", (unsigned)SPARKMAX_HEARTBEAT_ARBID);
    ESP_LOGI(TAG, "  duty cycle 0x%08X", (unsigned)SPARKMAX_ARBID(SPARKMAX_DUTY_CYCLE_BASE, motor->device_id));
    ESP_LOGI(TAG, "  velocity   0x%08X", (unsigned)SPARKMAX_ARBID(SPARKMAX_VELOCITY_BASE,   motor->device_id));
    ESP_LOGI(TAG, "  position   0x%08X", (unsigned)SPARKMAX_ARBID(SPARKMAX_POSITION_BASE,   motor->device_id));
    return true;
}

void sparkmax_end(sparkmax_t *motor)
{
    if (!motor->started) return;

    vTaskDelete(motor->hb_task);
    motor->hb_task = NULL;

    sparkmax_set_duty_cycle(motor, 0.0f);

    twai_node_disable(motor->node);
    twai_node_delete(motor->node);
    motor->node    = NULL;
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
    send_frame(motor, &motor->tx_frame_cmd, motor->tx_buf_cmd,
               SPARKMAX_ARBID(frame_base, motor->device_id), data, sizeof(data));
}
