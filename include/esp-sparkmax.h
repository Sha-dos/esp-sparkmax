#pragma once

/**
 * esp-sparkmax.h
 *
 * REV SPARK MAX CAN driver for ESP32 (plain C, IDF v5.4+).
 *
 * A background task sends the REV heartbeat (8×0xFF, arbId 0x02052C80)
 * every 10 ms. The SPARK MAX exits "no signal" (cyan blink) as soon as
 * it receives this frame.
 *
 * Quick start:
 *   sparkmax_config_t cfg = { .tx_pin = GPIO_NUM_6, .rx_pin = GPIO_NUM_5 };
 *   sparkmax_t motor;
 *   sparkmax_init(&motor, &cfg, 1);        // device ID 1 = factory default
 *   sparkmax_begin(&motor, NULL, NULL);
 *
 *   sparkmax_set_duty_cycle(&motor, 0.25f);
 *   vTaskDelay(pdMS_TO_TICKS(2000));
 *   sparkmax_set_duty_cycle(&motor, 0.0f);
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// CAN arbId constants  (REV FRC CAN, 29-bit extended)
//
// Formula:  arbId = (device_type << 24) | (manufacturer << 16)
//                 | (api_class << 10) | (api_index << 6) | device_id
//   device_type  = 0x02  (motor controller)
//   manufacturer = 0x05  (REV Robotics)
//
// Base arbIds have device_id = 0.  To address a specific motor, add its ID:
//   SPARKMAX_ARBID(SPARKMAX_DUTY_CYCLE_BASE, 1)  →  targets motor CAN ID 1
// ---------------------------------------------------------------------------

/* Control setpoint bases */
#define SPARKMAX_DUTY_CYCLE_BASE    0x02050080UL  /* apiClass=0,  apiIndex=2  */
#define SPARKMAX_VELOCITY_BASE      0x02050480UL  /* apiClass=1,  apiIndex=2  */
#define SPARKMAX_SMART_VEL_BASE     0x020504C0UL  /* apiClass=1,  apiIndex=3  */
#define SPARKMAX_POSITION_BASE      0x02050C80UL  /* apiClass=3,  apiIndex=2  */
#define SPARKMAX_VOLTAGE_BASE       0x02051080UL  /* apiClass=4,  apiIndex=2  */
#define SPARKMAX_CURRENT_BASE       0x020510C0UL  /* apiClass=4,  apiIndex=3  */
#define SPARKMAX_SMART_MOTION_BASE  0x02051480UL  /* apiClass=5,  apiIndex=2  */

/* Periodic status frame bases */
#define SPARKMAX_STATUS_0_BASE      0x02051800UL
#define SPARKMAX_STATUS_1_BASE      0x02051840UL
#define SPARKMAX_STATUS_2_BASE      0x02051880UL
#define SPARKMAX_STATUS_3_BASE      0x020518C0UL
#define SPARKMAX_STATUS_4_BASE      0x02051900UL

/* Heartbeat: broadcast, no device_id — all motors on the bus respond */
#define SPARKMAX_HEARTBEAT_ARBID    0x02052C80UL

/* Build an addressed arbId for a specific motor */
#define SPARKMAX_ARBID(base, dev_id)  ((base) + (uint32_t)(dev_id))

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/** CAN bus pin configuration. */
typedef struct {
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
} sparkmax_config_t;

/**
 * RX callback — called from ISR context on every received frame.
 * Must be short and non-blocking.
 */
typedef void (*sparkmax_rx_cb_t)(uint32_t arb_id,
                                  const uint8_t *data, uint8_t len,
                                  void *user_ctx);

/** SPARK MAX driver handle. Allocate on the stack or statically. */
typedef struct {
    sparkmax_config_t   config;
    uint8_t             device_id;
    bool                started;
    twai_node_handle_t  node;
    TaskHandle_t        hb_task;
    TaskHandle_t        decode_task;
    QueueHandle_t       rx_queue;
    sparkmax_rx_cb_t    rx_cb;
    void               *rx_user_ctx;

    /*
     * The TWAI driver holds a raw twai_frame_t* until the TX-done ISR fires,
     * so both the frame struct and its data buffer must be long-lived.
     * Separate slots for heartbeat, setpoint, and status config so they never collide.
     */
    uint8_t      tx_buf_hb[8];
    twai_frame_t tx_frame_hb;

    uint8_t      tx_buf_cmd[8];
    twai_frame_t tx_frame_cmd;

    uint8_t      tx_buf_status[8];
    twai_frame_t tx_frame_status;

    volatile float position_rotations;   /* Status 2: motor position (rotations) */
    volatile float velocity_rpm;         /* Status 1: motor velocity (RPM)        */
    volatile float bus_voltage;          /* Status 1: bus voltage (V)             */
    volatile float output_current;       /* Status 1: output current (A)          */
    volatile float applied_output;       /* Status 0: applied output [-1, 1]      */
    volatile float temperature_c;        /* Status 1: motor temperature (°C) */
} sparkmax_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

/**
 * Initialise a sparkmax_t handle (does not touch the bus).
 * Call before sparkmax_begin().
 */
void sparkmax_init(sparkmax_t *motor,
                   const sparkmax_config_t *config,
                   uint8_t device_id);

/**
 * Start the TWAI peripheral and the heartbeat task.
 * @param rx_cb    Optional ISR callback for received frames (NULL to ignore).
 * @param user_ctx Passed verbatim to rx_cb.
 * @return true on success.
 */
bool sparkmax_begin(sparkmax_t *motor,
                    sparkmax_rx_cb_t rx_cb,
                    void *user_ctx);

/** Stop the heartbeat task, zero the motor, and shut down TWAI. */
void sparkmax_end(sparkmax_t *motor);

/** Duty-cycle setpoint [-1.0, 1.0]. Clamped automatically. */
void sparkmax_set_duty_cycle(sparkmax_t *motor, float duty);

/** Velocity setpoint in RPM (default units). */
void sparkmax_set_velocity(sparkmax_t *motor, float rpm);

/** Position setpoint in rotations (default units). */
void sparkmax_set_position(sparkmax_t *motor, float rotations);

/**
 * Override the transmit period of a periodic status frame.
 * @param frame_base  One of SPARKMAX_STATUS_x_BASE.
 * @param period_ms   Desired period in milliseconds.
 */
void sparkmax_set_status_period(sparkmax_t *motor,
                                uint32_t frame_base,
                                uint16_t period_ms);

/**
 * Read the latest decoded periodic status values.
 * These are updated automatically by the RX ISR whenever the SPARK MAX
 * broadcasts its periodic frames.  Enable Status 2 first with:
 *   sparkmax_set_status_period(motor, SPARKMAX_STATUS_2_BASE, 20);
 */
float sparkmax_get_position(sparkmax_t *motor);     /* rotations          */
float sparkmax_get_velocity(sparkmax_t *motor);     /* RPM                */
float sparkmax_get_voltage(sparkmax_t *motor);      /* bus voltage (V)    */
float sparkmax_get_current(sparkmax_t *motor);      /* output current (A) */
float sparkmax_get_output(sparkmax_t *motor);       /* applied output [-1,1] */
float sparkmax_get_temperature(sparkmax_t *motor);  /* motor temp (°C) */

#ifdef __cplusplus
}
#endif

