#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Configuration ===================================================== */

#ifndef MF_MAX_MOTORS
#define MF_MAX_MOTORS 16
#endif

/* ===== Meta update flags (MF_GetMetaUpdates) ============================= */

#define MF_META_COUNT   (1u<<0)   /* N: motor count changed            */
#define MF_META_MODE    (1u<<1)   /* MODE: MANUAL/AUTO changed         */
#define MF_META_LOOP    (1u<<2)   /* LOOP: 0/1 changed                 */
#define MF_META_GSPEED  (1u<<3)   /* G: global ms/deg provided (>0)    */

/* ===== Types ============================================================= */

/* Operating mode */
typedef enum {
    MF_MODE_MANUAL = 0,
    MF_MODE_AUTO   = 1
} MF_Mode;

/* Per-motor desired state (angle in degrees, speed in ms/deg) */
typedef struct {
    uint16_t angle;              /* 0..180 */
    uint16_t speed_ms_per_deg;   /* 0 => ignore / unchanged */
} MF_Motor;

/* ===== Parser / protocol API ============================================ */

/* Initialise module state and parser */
void     MF_Init(void);

/* Feed bytes from UART (DMA+IDLE or polling). Safe with arbitrary chunking. */
void     MF_ProcessBytes(const uint8_t* data, size_t len);

/* ===== Per-motor state =================================================== */

/* Bitmask of motors updated by last A/V lists (bit i -> motor i) */
uint32_t MF_GetUpdatedMask(void);

/* Clear bits after your main loop consumes the updates */
void     MF_ClearUpdatedMask(uint32_t mask);

/* Snapshot of one motor’s target state */
MF_Motor MF_GetState(uint8_t idx);

/* ===== Meta (frame-level) =============================================== */

/* Active channels (default 8; set via N:) */
uint8_t  MF_GetMotorCount(void);

/* MANUAL or AUTO */
MF_Mode  MF_GetMode(void);

/*
 * Loop flag.
 * NOTE: When mode is AUTO, the implementation clamps LOOP to 0.
 * This getter also returns 0 while in AUTO (belt & suspenders).
 */
uint8_t  MF_GetLoop(void);

/* Last G: value in ms/deg. (>0 when present in last frame) */
uint16_t MF_GetGlobalSpeed(void);

/* Bitmask of MF_META_* that changed with the last committed frame */
uint8_t  MF_GetMetaUpdates(void);

/* Acknowledge/clear consumed meta bits */
void     MF_ClearMetaUpdates(uint8_t mask);

/* Optional helper for logs/telemetry (names for first 8 channels) */
const char* MF_MotorName(uint8_t idx);

#ifdef __cplusplus
}
#endif
