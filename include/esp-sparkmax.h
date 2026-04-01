#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_twai.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// CAN arbId constants
// ---------------------------------------------------------------------------
#define SPARKMAX_DUTY_CYCLE_BASE    0x02050080UL
#define SPARKMAX_VELOCITY_BASE      0x02050480UL
#define SPARKMAX_SMART_VEL_BASE     0x020504C0UL
#define SPARKMAX_POSITION_BASE      0x02050C80UL
#define SPARKMAX_VOLTAGE_BASE       0x02051080UL
#define SPARKMAX_CURRENT_BASE       0x020510C0UL
#define SPARKMAX_SMART_MOTION_BASE  0x02051480UL

#define SPARKMAX_STATUS_0_BASE      0x02051800UL
#define SPARKMAX_STATUS_1_BASE      0x02051840UL
#define SPARKMAX_STATUS_2_BASE      0x02051880UL
#define SPARKMAX_STATUS_3_BASE      0x020518C0UL
#define SPARKMAX_STATUS_4_BASE      0x02051900UL

#define SPARKMAX_HEARTBEAT_ARBID    0x02052C80UL
#define SPARKMAX_ARBID(base, dev_id) ((base) + (uint32_t)(dev_id))

// ---------------------------------------------------------------------------
// User-provided CAN transport — required, set in sparkmax_config_t
// ---------------------------------------------------------------------------

/**
 * Transmit a CAN frame. Called from task context (not ISR).
 * Must be non-NULL before calling sparkmax_begin().
 *
 * @param arb_id  29-bit extended CAN ID.
 * @param data    Payload bytes (up to 8).
 * @param len     Payload length.
 * @return ESP_OK on success, any esp_err_t on failure.
 */
typedef esp_err_t (*sparkmax_tx_fn)(uint32_t arb_id,
                                    const uint8_t *data,
                                    uint8_t len,
                                    void *user_ctx);

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

typedef struct {
    sparkmax_tx_fn  tx_fn;      /**< Required: transmit callback.          */
    void           *tx_ctx;     /**< Passed verbatim to tx_fn.             */
} sparkmax_config_t;

typedef void (*sparkmax_rx_cb_t)(uint32_t arb_id,
                                  const uint8_t *data, uint8_t len,
                                  void *user_ctx);

typedef struct {
    sparkmax_config_t   config;
    uint8_t             device_id;
    bool                started;
    TaskHandle_t        hb_task;
    TaskHandle_t        decode_task;
    QueueHandle_t       rx_queue;
    sparkmax_rx_cb_t    rx_cb;
    void               *rx_user_ctx;

    volatile float position_rotations;
    volatile float velocity_rpm;
    volatile float bus_voltage;
    volatile float output_current;
    volatile float applied_output;
    volatile float temperature_c;
} sparkmax_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

void sparkmax_init(sparkmax_t *motor,
                   const sparkmax_config_t *config,
                   uint8_t device_id);

bool sparkmax_begin(sparkmax_t *motor,
                    sparkmax_rx_cb_t rx_cb,
                    void *user_ctx);

void sparkmax_end(sparkmax_t *motor);

/**
 * Feed a received CAN frame into the driver.
 * Call this for every frame your RX ISR or task receives — the driver
 * filters by arb_id internally and ignores unrelated frames.
 * Safe to call from ISR context.
 */
void sparkmax_feed_rx(sparkmax_t *motor,
                      uint32_t arb_id,
                      const uint8_t *data,
                      uint8_t len);

void sparkmax_set_duty_cycle(sparkmax_t *motor, float duty);
void sparkmax_set_velocity(sparkmax_t *motor, float rpm);
void sparkmax_set_position(sparkmax_t *motor, float rotations);
void sparkmax_set_status_period(sparkmax_t *motor,
                                uint32_t frame_base,
                                uint16_t period_ms);

float sparkmax_get_position(sparkmax_t *motor);
float sparkmax_get_velocity(sparkmax_t *motor);
float sparkmax_get_voltage(sparkmax_t *motor);
float sparkmax_get_current(sparkmax_t *motor);
float sparkmax_get_output(sparkmax_t *motor);
float sparkmax_get_temperature(sparkmax_t *motor);

#ifdef __cplusplus
}
#endif