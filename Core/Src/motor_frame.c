/*
 * motor_frame.c  (Updated)
 *
 * Streaming UART frame parser for STM32 Bluepill slave controller.
 *
 * Protocol (order-free, any subset per frame):
 *   < A:a1,...,aN; V:s1,...,sN; G:ms; N:n; MODE:MANUAL|AUTO; LOOP:0|1; *CS>\n
 *
 *  - CS is the XOR of all bytes between '<' and '*' (exclusive), written as two
 *    uppercase hex digits (00..FF).
 *  - Sections are optional and may appear in any order. A section may be the
 *    final field before '*' (no trailing ';') — this file now safely handles that.
 *
 * Behavioural notes:
 *  - Angles (A) are clamped to 0..180. Each provided entry marks that motor
 *    "updated" in a bitmask returned by MF_GetUpdatedMask().
 *  - Per-finger speeds (V) are latched as ms/deg (0 means "ignore / leave
 *    unchanged").
 *  - Global speed (G) is considered "present" only if > 0; the main loop
 *    applies it uniformly when MF_META_GSPEED is raised.
 *  - Motor count (N) is clamped to [1..MF_MAX_MOTORS].
 *  - MODE text is latched when a delimiter (';' or ',') is seen. In addition,
 *    we now finalize MODE if the frame ends immediately after the token
 *    (i.e., just before '*').
 *  - LOOP is 0/1.
 *  - ENFORCEMENT: If MODE==AUTO, LOOP is forced to 0 at commit time, and an
 *    MF_META_LOOP update is raised if clamped. (Matches requested invariant.)
 *
 * Integration points used elsewhere:
 *  - uart_dma.c calls:    MF_Init(), MF_ProcessBytes(const uint8_t*, size_t)
 *  - main.c reads state:  MF_GetUpdatedMask/Clear, MF_GetState(idx)
 *                          MF_GetMotorCount/Mode/Loop/GlobalSpeed
 *                          MF_GetMetaUpdates/Clear
 *
 * This implementation favours robustness and clear comments over micro-optimisation.
 */

#include <string.h>
#include <ctype.h>
#include "motor_frame.h"

/* ===== Types ============================================================= */

#ifndef MF_DEFAULT_COUNT
#define MF_DEFAULT_COUNT 8
#endif

/* NOTE: The public MF_Motor and MF_Mode are declared in motor_frame.h. If your
 * header is older, it should look like:
 *
 *   typedef struct { uint16_t angle; uint16_t speed_ms_per_deg; } MF_Motor;
 *   typedef enum { MF_MODE_MANUAL=0, MF_MODE_AUTO=1 } MF_Mode;
 */

/* Internal parser state */
typedef enum {
    FR_IDLE = 0,   /* outside any frame */
    FR_BODY,       /* inside body between '<' and '*' */
    FR_CS1,        /* expecting first hex digit of checksum */
    FR_CS2         /* expecting second hex digit of checksum */
} FrState;

typedef enum {
    SEC_NONE = 0,
    SEC_A,         /* angles list */
    SEC_V,         /* per-finger speeds list */
    SEC_G,         /* global ms/deg */
    SEC_N,         /* motor count */
    SEC_MODE,      /* text token: AUTO|MANUAL */
    SEC_LOOP       /* numeric 0|1 */
} Section;

/* One running frame parse */
typedef struct {
    FrState st;
    uint8_t xsum;                    /* XOR of body bytes */
    uint8_t cs_read;                 /* checksum we've parsed (binary) */
    char    tag[8];                  /* current section tag (e.g., "A","MODE") */
    uint8_t taglen;

    Section sec;                     /* which section we're inside */
    uint8_t idx;                     /* list index for A/V */
    int32_t num;                     /* accumulating number */
    uint8_t have_num;                /* are we in a numeric token */

    char    keybuf[10];              /* for MODE token ("AUTO"/"MANUAL") */
    uint8_t keylen;

    /* Temporary results to be committed at checksum OK */
    uint8_t  have_mode;
    MF_Mode  mode_tmp;

    uint8_t  have_loop;
    uint8_t  loop_tmp;

    uint8_t  have_g;
    uint16_t gspeed;

    uint8_t  have_n;
    uint8_t  n_tmp;

    /* For A/V we update s_motor/s_updated_mask on the fly inside the frame. */
} FrameRun;

/* ===== Module state ====================================================== */

/* Per-motor desired state (angle, ms/deg). angle in 0..180, speed 0:ignore */
static MF_Motor s_motor[MF_MAX_MOTORS];
/* Bit i set => motor i updated by latest A/V */
static volatile uint32_t s_updated_mask = 0;

/* Meta (frame-level) state controlled by framed sections */
static uint8_t  s_motor_count = MF_DEFAULT_COUNT; /* active channels */
static MF_Mode  s_mode        = MF_MODE_MANUAL;
static uint8_t  s_loop        = 0;
static uint16_t s_global_speed= 0;
/* MF_META_* bitfield of pending meta updates */
static volatile uint8_t s_meta_updates = 0;

/* Running parser instance */
static FrameRun fr;

/* ===== Utilities ========================================================= */

static inline uint16_t clamp_u16(int v, int lo, int hi)
{ if (v < lo) v = lo; if (v > hi) v = hi; return (uint16_t)v; }

static inline uint16_t clampElbow_u16(int v, int lo, int hi)
{ if (v < lo) v = lo; if (v > hi) v = hi; return (uint16_t)v; }

static inline uint8_t hex2nib(int c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFF;
}

/* Commit a numeric token depending on current section */
static void fr_finalize_number(void)
{
    if (!fr.have_num) return;

    switch (fr.sec) {
    case SEC_A: {
        /* Write next angle entry and set updated bit */
        uint8_t i = fr.idx;
        if (i < MF_MAX_MOTORS) {
        	if(i == 6) {s_motor[i].angle = clampElbow_u16(fr.num, 0, 50);}
        	else{
            s_motor[i].angle = clamp_u16(fr.num, 0, 180);}
            /* NOTE: leave individual speed unchanged here; main decides final rate */
            s_updated_mask |= (1u << i);
            fr.idx++; /* advance index for next list entry */
        }
    } break;
    case SEC_V: {
        uint8_t i = fr.idx;
        if (i < MF_MAX_MOTORS) {
            /* 0 => ignore/leave unchanged; otherwise 1..1000 typical */
            uint16_t sp = (fr.num < 0) ? 0 : (fr.num > 2000 ? 2000 : (uint16_t)fr.num);
            s_motor[i].speed_ms_per_deg = sp;
            /* touching speed also counts as "updated" so main can latch per-finger */
            s_updated_mask |= (1u << i);
            fr.idx++;
        }
    } break;
    case SEC_G: {
        /* Only >0 counts as "present" for meta update */
        uint16_t sp = (fr.num < 0) ? 0 : (fr.num > 2000 ? 2000 : (uint16_t)fr.num);
        fr.gspeed  = sp;
        fr.have_g  = (sp > 0) ? 1 : 0;
    } break;
    case SEC_N: {
        uint8_t n = (fr.num < 1) ? 1 : (fr.num > MF_MAX_MOTORS ? MF_MAX_MOTORS : (uint8_t)fr.num);
        fr.n_tmp   = n;
        fr.have_n  = 1;
    } break;
    case SEC_LOOP: {
        fr.loop_tmp  = (fr.num != 0) ? 1 : 0;
        fr.have_loop = 1;
    } break;
    default: break;
    }

    fr.num = 0;
    fr.have_num = 0;
}

/* Commit a MODE token sitting in keybuf (AUTO/MANUAL) */
static void fr_finalize_mode_if_pending(void)
{
    if (fr.sec == SEC_MODE && fr.keylen > 0) {
        fr.keybuf[fr.keylen] = 0;
        if      (strcmp(fr.keybuf, "AUTO")   == 0) { fr.mode_tmp = MF_MODE_AUTO;   fr.have_mode = 1; }
        else if (strcmp(fr.keybuf, "MANUAL") == 0) { fr.mode_tmp = MF_MODE_MANUAL; fr.have_mode = 1; }
        /* else: unknown token => ignore silently */

        /* reset key */
        fr.keylen = 0;
        fr.keybuf[0] = 0;
    }
}

/* Reset section/tag state between sections or at frame start */
static void fr_reset_section(void)
{
    fr.sec    = SEC_NONE;
    fr.taglen = 0;
    fr.idx    = 0;
    fr.num    = 0;
    fr.have_num = 0;
    fr.keylen = 0;
    fr.keybuf[0] = 0;
}

/* Reset frame runner for a new frame */
static void fr_begin(void)
{
    memset(&fr, 0, sizeof(fr));
    fr.st   = FR_BODY;
    fr.xsum = 0;
    fr.sec  = SEC_NONE;
}

/* ===== Public API ======================================================== */

void MF_Init(void)
{
    memset(s_motor, 0, sizeof(s_motor));
    s_updated_mask = 0;
    s_motor_count  = MF_DEFAULT_COUNT;
    s_mode         = MF_MODE_MANUAL;
    s_loop         = 0;
    s_global_speed = 0;
    s_meta_updates = 0;
    memset(&fr, 0, sizeof(fr));
    fr.st = FR_IDLE;
}

/* Streaming byte feeder (DMA+IDLE calls this with arbitrary chunks) */
void MF_ProcessBytes(const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        uint8_t b = data[i];

        /* Legacy (brief): accept single-record "F#:ang,speed;" (or W/E/S aliases).
         * This clause is intentionally minimal; framed packets are preferred. */
        if (fr.st == FR_IDLE) {
            if (b == '<') { fr_begin(); continue; }
            /* (Legacy parsing could go here if needed) */
            continue;
        }

        switch (fr.st) {
        case FR_BODY: {
            if (b == '*') {
                /* End of body: finalize any pending tokens, then move to CS */
                fr_finalize_mode_if_pending();
                fr_finalize_number();
                fr.st = FR_CS1;
                break;
            }

            /* Maintain running XOR of body bytes */
            fr.xsum ^= b;

            if (fr.sec == SEC_NONE) {
                /* We are expecting a TAG like 'A', 'V', 'G', 'N', 'MODE', 'LOOP' until ':' */
                if (b == ';' || b == ',') {
                    /* stray delimiter; ignore */
                } else if (b == ':') {
                    /* Resolve tag -> section */
                    fr.tag[fr.taglen] = 0;
                    if      (fr.taglen == 1 && fr.tag[0] == 'A') fr.sec = SEC_A;
                    else if (fr.taglen == 1 && fr.tag[0] == 'V') fr.sec = SEC_V;
                    else if (fr.taglen == 1 && fr.tag[0] == 'G') fr.sec = SEC_G;
                    else if (fr.taglen == 1 && fr.tag[0] == 'N') fr.sec = SEC_N;
                    else if (strcmp(fr.tag, "MODE") == 0)        fr.sec = SEC_MODE;
                    else if (strcmp(fr.tag, "LOOP") == 0)        fr.sec = SEC_LOOP;
                    else                                         fr.sec = SEC_NONE; /* unknown -> ignore values */

                    /* reset per-section variables */
                    fr.idx = 0;
                    fr.num = 0;
                    fr.have_num = 0;
                    fr.keylen = 0;
                } else {
                    /* collect tag letters (up to buffer size) */
                    if (fr.taglen < sizeof(fr.tag)-1) fr.tag[fr.taglen++] = (char)b;
                }
                break;
            }

            /* Within a section */
            if (b == ';') {
                /* Section terminator: finalize content and reset */
                fr_finalize_mode_if_pending();
                fr_finalize_number();
                fr_reset_section();
                break;
            }

            if (fr.sec == SEC_A || fr.sec == SEC_V) {
                if (b == ',') {
                    fr_finalize_number();
                } else if (b == '-' || (b >= '0' && b <= '9')) {
                    /* numeric: allow '-' only at start */
                    if (b == '-') {
                        if (!fr.have_num) { fr.have_num = 1; fr.num = 0; }
                        /* negative -> mark sign; we will clamp anyway */
                    } else {
                        if (!fr.have_num) { fr.have_num = 1; fr.num = 0; }
                        fr.num = fr.num*10 + (b - '0');
                    }
                } else {
                    /* ignore */
                }
            } else if (fr.sec == SEC_G || fr.sec == SEC_N || fr.sec == SEC_LOOP) {
                if (b == ',' ) {
                    fr_finalize_number();
                } else if (b >= '0' && b <= '9') {
                    if (!fr.have_num) { fr.have_num = 1; fr.num = 0; }
                    fr.num = fr.num*10 + (b - '0');
                } else {
                    /* ignore */
                }
            } else if (fr.sec == SEC_MODE) {
                /* MODE expects letters of "AUTO" or "MANUAL" */
                if (b == ',' ) {
                    fr_finalize_mode_if_pending();
                } else if (isalpha((int)b)) {
                    if (fr.keylen < sizeof(fr.keybuf)-1) {
                        fr.keybuf[fr.keylen++] = (char)toupper((int)b);
                    }
                } else {
                    /* ignore */
                }
            }
        } break;

        case FR_CS1: {
            uint8_t n = hex2nib(b);
            if (n == 0xFF) { fr.st = FR_IDLE; break; }
            fr.cs_read = (uint8_t)(n << 4);
            fr.st = FR_CS2;
        } break;

        case FR_CS2: {
            uint8_t n = hex2nib(b);
            if (n == 0xFF) { fr.st = FR_IDLE; break; }
            fr.cs_read |= n;
            /* Frame should end with '>' or '\n' (we accept both orders) — skip checking; commit immediately */
            /* Compare checksum and commit if OK */
            if (fr.cs_read == fr.xsum) {
                /* Apply meta: MODE/LOOP/N/G (angles/speeds already touched s_motor/s_updated_mask) */
                if (fr.have_mode) {
                    s_mode = fr.mode_tmp;
                    s_meta_updates |= MF_META_MODE;
                }
                if (fr.have_loop) {
                    s_loop = (fr.loop_tmp != 0) ? 1 : 0;
                    s_meta_updates |= MF_META_LOOP;
                }
                if (fr.have_n) {
                    s_motor_count = fr.n_tmp;
                    s_meta_updates |= MF_META_COUNT;
                }
                if (fr.have_g) {
                    s_global_speed = fr.gspeed;
                    s_meta_updates |= MF_META_GSPEED;
                }

                /* Enforce invariant: AUTO => LOOP=0 (and advertise if clamped) */
                if (s_mode == MF_MODE_AUTO && s_loop != 0) {
                    s_loop = 0;
                    s_meta_updates |= MF_META_LOOP;
                }
            }
            /* Reset to idle for next frame */
            fr.st = FR_IDLE;
        } break;

        default:
            fr.st = FR_IDLE;
            break;
        } /* switch */
    } /* for */
}

/* ===== State getters/setters ============================================ */

uint32_t MF_GetUpdatedMask(void)           { return s_updated_mask; }
void     MF_ClearUpdatedMask(uint32_t m)   { s_updated_mask &= ~m;  }
MF_Motor MF_GetState(uint8_t idx)
{
    MF_Motor z = {0,0};
    if (idx >= MF_MAX_MOTORS) return z;
    return s_motor[idx];
}

/* Meta */
uint8_t  MF_GetMotorCount(void)            { return s_motor_count; }
MF_Mode  MF_GetMode(void)                  { return s_mode; }
uint8_t  MF_GetLoop(void)                  { return (s_mode == MF_MODE_AUTO) ? 0 : s_loop; } /* belt & suspenders */
uint16_t MF_GetGlobalSpeed(void)           { return s_global_speed; }
uint8_t  MF_GetMetaUpdates(void)           { return s_meta_updates; }
void     MF_ClearMetaUpdates(uint8_t mask) { s_meta_updates &= (uint8_t)~mask; }

/* Optional friendly names for debugging/telemetry */
const char* MF_MotorName(uint8_t idx)
{
    static const char* k8[8] = {
        "F1(Thumb)","F2(Index)","F3(Middle)","F4(Ring)",
        "F5(Little)","Wrist","Elbow","Shoulder"
    };
    if (idx < 8) return k8[idx];
    return "?";
}
