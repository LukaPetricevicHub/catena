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

#define CATENA_NONE 0xFFu      /* "no robot" in id fields */
#define CATENA_STROKE_POINTS 32 /* points of a drawn stroke (spec 7.4) */

/* Frame types: first payload byte. Link frames go to one robot; core drops every frame
 * whose sender it has not seen flashing within radio.neighbour_timeout. */
#define CATENA_FRAME_FLASH 1u    /* broadcast: [type, id, chain id, size, index, flags, target, ahead, behind] */
#define CATENA_FRAME_LINK_REQ 2u /* head to tail while driving in: [type, chain id, size] */
#define CATENA_FRAME_LINK_ACK 3u /* hub robot to the robot on its hub, every step the link holds:
                                    [type, index, lowest id from head to sender, hall, speed] */
#define CATENA_FRAME_LINK_UP 4u  /* linked robot to the robot ahead, every step:
                                    [type, robots from sender to tail, lowest id among them,
                                    flags (1 hold still, 2 a higher-ranked chain is in sight), slack] */
#define CATENA_FRAME_GUIDE 5u    /* tail to a seeker: [type, seeker's offset from the middle of the
                                    acceptance zone, int8, 127 = pi, counter-clockwise seen from above] */
#define CATENA_FRAME_LINK_NAK 6u /* hub robot to the robot on its hub: back out (same chain) */
#define CATENA_FRAME_SHAPE 7u    /* between linked robots, one per step: [type, shape version,
                                    flags (1 closed, 2 active, bits 4-5 chunk number), twist
                                    sequence, vx, vy (int8 mm/s), wz (int8, 0.01 rad/s), edge push
                                    x, y (int8 mm/s, shape frame), then 8 stroke points (x, y)] */
#define CATENA_FRAME_CLOSE 8u    /* head and tail of a closed ring, every step: [type, turning
                                    error (int16), tongue direction (from the head) or hall
                                    reading (from the tail) (int16)] */
/* While a shape is held LINK_ACK and LINK_UP carry two more bytes, the sender's turning
 * error, and LINK_UP two more, its tongue direction. int16, little endian, 1e-4 rad. */

/* FLASH flags. */
#define CATENA_FLAG_HEAD 1u    /* tongue free */
#define CATENA_FLAG_TAIL 2u    /* hub free */
#define CATENA_FLAG_DOCKING 4u /* lining up or driving in on `target` */

/* debug.state codes. */
#define CATENA_STATE_WANDER 1u
#define CATENA_STATE_SEEK 2u
#define CATENA_STATE_DOCK 3u
#define CATENA_STATE_FOLLOW 4u
#define CATENA_STATE_SHAPE 5u  /* holding the drawn shape */
#define CATENA_STATE_MOVE 6u   /* shape plus twist */
/* debug.detail codes while holding a shape. */
#define CATENA_SHAPE_HOLD 0u
#define CATENA_SHAPE_CLOSING 1u /* the head docks onto its own tail */
#define CATENA_SHAPE_OPENING 2u /* the head pulls out of its own tail */
#define CATENA_SHAPE_EDGE 3u    /* backing away from a table edge */
/* debug.detail codes while wandering. */
#define CATENA_WANDER_CRUISE 0u
#define CATENA_WANDER_AVOID 1u
#define CATENA_WANDER_BACKOFF 2u
#define CATENA_WANDER_TURN 3u
#define CATENA_WANDER_HOLD 4u  /* standing still while a robot docks onto the chain's tail */
/* debug.detail codes while seeking. */
#define CATENA_SEEK_TAIL 0u    /* going to a tail */
#define CATENA_SEEK_TRACE 1u   /* following a chain's members toward its tail */
#define CATENA_SEEK_LEAVE 2u   /* backing off after failed docking attempts */
/* debug.detail codes while docking. */
#define CATENA_DOCK_ALIGN 0u
#define CATENA_DOCK_DRIVE 1u
#define CATENA_DOCK_RETREAT 2u

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
  uint8_t index;              /* position in the chain; the head is 0 */
  uint8_t chain_size;
  uint8_t neighbours;         /* robots measured by flash within the trust window */
  uint8_t chain_id;           /* lowest robot id in the chain, as far as gossip has reached */
  uint8_t ahead;              /* robot whose hub my tongue sits on, or CATENA_NONE */
  uint8_t behind;             /* robot whose tongue sits on my hub, or CATENA_NONE */
  uint8_t target;             /* tail being sought or docked onto, or CATENA_NONE */
  uint8_t shape_version;      /* 0: no shape received */
  uint8_t closure;            /* other end of the ring's closing link, or CATENA_NONE */
  int8_t shape_error_deg;     /* my turning angle minus its target */
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

/* What the phone hands to the robot it has joined; that robot passes it along its links.
 * xy: CATENA_STROKE_POINTS points as x0, y0, x1, ... in pad units 0..255, y up.
 * active 0 releases the chain back to FOLLOW. */
CATENA_API void catena_set_shape(catena_t* robot, const uint8_t* xy, int closed, int active);
/* Twist in the shape frame (spec 7.5). */
CATENA_API void catena_set_twist(catena_t* robot, float vx_mm_s, float vy_mm_s, float wz_rad_s);

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
