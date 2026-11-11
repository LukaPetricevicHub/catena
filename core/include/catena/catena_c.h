/* C API of the Catena core. The simulator loads it with ctypes; the firmware links it.
 * sim/corelib.py mirrors every struct here field by field: change both together. */
#ifndef CATENA_C_H
#define CATENA_C_H

#include <stddef.h>
#include <stdint.h>

#include "catena/params_gen.h" /* catena_params_t, generated from config/robot.yaml */

#if defined(__GNUC__)
#define CATENA_API __attribute__((visibility("default")))
#else
#define CATENA_API
#endif

#define CATENA_FRAME_MAX 32    /* bytes of payload per radio frame */
#define CATENA_RX_MAX 16       /* frames delivered per step; the platform drops the rest */
#define CATENA_TX_MAX 4        /* unicast frames sent per step */
#define CATENA_BROADCAST 0xFFu /* destination id of a broadcast */

/* Frame types: first payload byte. */
#define CATENA_FRAME_FLASH 1u /* broadcast; [type, sender id] */

/* debug.state codes. */
#define CATENA_STATE_WANDER 1u
/* debug.detail codes while wandering. */
#define CATENA_WANDER_CRUISE 0u
#define CATENA_WANDER_AVOID 1u
#define CATENA_WANDER_BACKOFF 2u
#define CATENA_WANDER_TURN 3u

#ifdef __cplusplus
extern "C" {
#endif

/* One received radio frame. Receiver k of the IR ring points at k * 360 / CATENA_IR_COUNT
 * degrees in the base frame, counter-clockwise from forward. */
typedef struct catena_rx_t {
  uint8_t src;                /* sender id from the radio layer */
  uint8_t len;                /* payload bytes in data */
  uint8_t has_ir;             /* FLASH only: 1 if the IR pulse was sampled cleanly */
  uint8_t reserved;
  float ir[CATENA_IR_COUNT];  /* pulse minus ambient per receiver; 1.0 = facing, at reference range */
  uint8_t data[CATENA_FRAME_MAX];
} catena_rx_t;

typedef struct catena_inputs_t {
  uint32_t dt_us;             /* time since the previous step */
  uint32_t entropy;           /* fresh random bits from the platform */
  float gyro_z_rad_s;         /* base yaw rate, counter-clockwise positive */
  float ring_angle_rad;       /* tongue direction relative to the base, counter-clockwise */
  float hall;                 /* 0..1; 1 = a tongue tip sits on my hub */
  int32_t ticks_left;         /* wheel stripe counts since start, signed by direction */
  int32_t ticks_right;
  uint8_t table_left;         /* edge sensors: 1 = table under the spot */
  uint8_t table_right;
  uint8_t rx_count;
  uint8_t reserved;
  catena_rx_t rx[CATENA_RX_MAX];
} catena_inputs_t;

typedef struct catena_tx_t {
  uint8_t dst;                /* robot id or CATENA_BROADCAST */
  uint8_t len;
  uint8_t reserved[2];
  uint8_t data[CATENA_FRAME_MAX];
} catena_tx_t;

/* Read only by the viewer and tests. */
typedef struct catena_debug_t {
  uint8_t state;              /* CATENA_STATE_* */
  uint8_t detail;             /* sub-state, e.g. CATENA_WANDER_* */
  uint8_t index;              /* position in the chain; 0 until chains exist */
  uint8_t chain_size;
  uint8_t neighbours;         /* robots measured by flash within the trust window */
  uint8_t reserved[3];
} catena_debug_t;

typedef struct catena_outputs_t {
  float wheel_left_mm_s;
  float wheel_right_mm_s;
  uint8_t flash;              /* 1: broadcast flash_data, then pulse the IR LEDs */
  uint8_t flash_len;
  uint8_t tx_count;
  uint8_t reserved;
  uint32_t flash_delay_us;    /* when to send the FLASH frame, from the start of this step */
  uint8_t flash_data[CATENA_FRAME_MAX];
  catena_tx_t tx[CATENA_TX_MAX];
  catena_debug_t debug;
} catena_outputs_t;

typedef struct catena_t catena_t;

/* Parameters compiled in from config/robot.yaml. */
CATENA_API void catena_default_params(catena_params_t* out);

/* One robot. Allocates once; returns NULL if out of memory. */
CATENA_API catena_t* catena_create(uint8_t robot_id, const catena_params_t* params);
CATENA_API void catena_step(catena_t* robot, const catena_inputs_t* in, catena_outputs_t* out);
CATENA_API void catena_destroy(catena_t* robot);

/* Flash ranging: bearing (rad, base frame) and centre distance (mm) from one IR sample.
 * Returns 0 if the sample holds no signal. */
CATENA_API int catena_flash_estimate(const catena_params_t* params, const float ir[CATENA_IR_COUNT],
                                     float* bearing_rad, float* range_mm);

/* Let a binding check that its struct layouts match this build. */
CATENA_API size_t catena_sizeof_params(void);
CATENA_API size_t catena_sizeof_inputs(void);
CATENA_API size_t catena_sizeof_outputs(void);

#ifdef __cplusplus
}
#endif

#endif
