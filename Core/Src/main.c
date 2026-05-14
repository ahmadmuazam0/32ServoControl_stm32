/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Bluepill STM32F103 — Slave controller (UART1 DMA + framed control)
  *
  * Features:
  *  - UART1 RX via DMA + IDLE (ISR feeds streaming parser)
  *  - Frames: A (angles), V (per-finger ms/deg), A+V, G (global ms/deg)
  *  - Flags: MODE:(MANUAL|AUTO), LOOP:(0|1), N:(motor count up to MAX_MOTORS)
  *  - Step-per-degree motion (default 30 ms/deg), per-finger latching on V>0
  *  - Watchdog -> glide to SAFE pose if master silent (AUTO mode only)
  *  - UART2 telemetry: ETA:ms on new move, DONE on arrival, ERR:WATCHDOG
  *
  * Master: Raspberry Pi / Node-RED (or any host)
  * Slave : STM32 “Bluepill”
  ******************************************************************************
  ***********Robotic Arm Motorized- Genaric Hand control Via Node-RED***********
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "uart_dma.h"     // U1RX_Init / U1RX_Start / U1RX_OnUsartIrq
#include "motor_frame.h"  // updated streaming parser API (A,V,G,N,MODE,LOOP + legacy)
#include "PCA9685.h"
#include "log_uart.h"		// System logging on UART2

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t angle;             // target angle (0..180)
  uint16_t speed_ms_per_deg;  // ms/degree (never 0 here; 0 only means "ignore" on input)
} FingerData;

typedef struct {
  uint16_t current;           // current servo angle
  uint16_t target;            // target servo angle
  uint16_t step_delay_ms;     // ms/degree for this channel
  uint32_t next_step_ms;      // next scheduling tick (HAL_GetTick-based)
} FingerMotion;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_NUM				8u	// Number of servos by default
#define MAX_MOTORS              16u      // compile-time ceiling to bound RAM
#define UART2_TX_TIMEOUT_MS     5u
#define TX_THROTTLE_MS          20u
#define DEFAULT_STEP_DELAY_MS   20u      // default speed = 20 ms/deg
#define SERVO_MIN_ANGLE         0
#define SERVO_MAX_ANGLE         180
#define WATCHDOG_MS             800u     // no valid frame -> failsafe
#define I2C_WRITES_PER_LOOP     6u   // how many PCA writes we allow per loop
#define STALL_MS                800u // no movement progress for this long => recover

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t   	g_motor_count = 8;                // runtime channels (clamped to SERVO_NUM)
static uint16_t 	g_global_ms_per_deg = DEFAULT_STEP_DELAY_MS;
static FingerData   g_finger[MAX_MOTORS];
static FingerMotion g_motion[MAX_MOTORS];
static uint16_t     g_servo_id[MAX_MOTORS];
static uint8_t      g_speed_latched[MAX_MOTORS];   // 0 = default, 1 = explicit speed set

// Safe pose (customize per build if desired)
static uint16_t SAFE_POSE[MAX_MOTORS] = { 0,0,0,0,0,90,0,0,  0,0,0,0,0,0,0,0 };

// For AUTO + LOOP ping-pong
static uint16_t g_last_cmd_angles[MAX_MOTORS];
static uint8_t  g_have_last_cmd = 0;
static uint8_t  g_auto_flip = 0;

// Timers/state
static uint32_t s_last_tx_ms  = 0;                 // telemetry throttle
static uint32_t s_last_cmd_ms = 0;                 // last command time (watchdog)
static uint32_t s_move_start_ms = 0;               // Move start timestamp
static uint8_t  s_step_active = 0;                 // motion in progress
static uint8_t  s_in_failsafe = 0;                 // failsafe already engaged
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static inline uint16_t clamp_u16(int v, int lo, int hi){
	v = (v < lo) ? lo : (v > hi ? hi : v);
	    return (uint16_t)v;
}
static void send_line(UART_HandleTypeDef* huart,const char *s);
static void send_snapshot_uart2(void);
static void Motion_Init(uint8_t count);
static void Motion_Update(uint8_t count);
static uint8_t all_reached(uint8_t count);
static void apply_targets_from_parser(uint32_t mask, uint8_t count, uint8_t *out_touched_angles);
static void handle_meta_updates(void);

// Make the recover available to main.c
void PCA9685_BusRecover(void) {
  HAL_I2C_DeInit(&hi2c1);
  HAL_Delay(2);
  MX_I2C1_Init();
  HAL_Delay(1);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  LOG_Init(&huart2);
  /* Boot banner */
  LOGI("Booting Bluepill slave (UART1 DMA+CIRCULAR, I2C1 PB6/PB7) - Version V1.1");



  U1RX_Init(&huart1);   // config UART1 for DMA circular + IDLE
  U1RX_Start();         // starts HAL_UART_Receive_DMA + enables UART_IT_IDLE
  LOGI("Init PCA9685 @ 50Hz");
    PCA9685_Init(50);

    /* Determine initial motor count (clamped to hardware) */
    uint8_t req = MF_GetMotorCount();                 // parser default=8 before any N:
    g_motor_count = MIN(MAX_MOTORS, (req ? req : g_motor_count));
    if (g_motor_count == 0) g_motor_count = 8;        // safe boot default


      /* Initialize the outputs for the available channels */
      for (uint8_t i = 0; i < g_motor_count; i++) {
        g_servo_id[i] = i;	// Intilaize the Instance for Motors 0-15

//	char line[32];
//			int n = snprintf(line, sizeof(line), "Servo Instance:%u Init\n", i);
//			if (n>0) send_line(&huart2, line);
			LOGI("Servo Instance initialized %u \n",g_servo_id[i]);
      }

      /* Initialize local motion state */
      Motion_Init(g_motor_count);


      /* Visual heartbeat */
      for (int i = 0; i < 6; i++) { HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); HAL_Delay(80); }

      s_last_cmd_ms = HAL_GetTick();
      s_in_failsafe = 0;
      LOGI("System Started\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	LOG_Poll();   // keep UART2 DMA moving
//	LOGI("System While Loop Start\n");
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 1) Handle meta updates (MODE / LOOP / N:count / G:global speed) */
    handle_meta_updates();
    /* 2) Handle per-motor updates (A / V / A+V / legacy F-records) */
    uint8_t touched_angles = 0;
    uint32_t mask = MF_GetUpdatedMask();
    if (mask) {

      apply_targets_from_parser(mask, g_motor_count, &touched_angles);
      MF_ClearUpdatedMask(mask);

      // Mark timing/status for this batch
      s_step_active = 1;
      s_move_start_ms = s_last_cmd_ms = HAL_GetTick();
      s_in_failsafe = 0;

       uint32_t now = HAL_GetTick();
      if ((now - s_last_tx_ms) >= TX_THROTTLE_MS) {
        send_snapshot_uart2();
        LOGD("Snapshot: %u motors; F1:%u,%u ...", g_motor_count, g_motion[0].target, g_motion[0].step_delay_ms);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        s_last_tx_ms = now;
      }
    }

    /* 3) Step the motion planner (non-blocking) */
    Motion_Update(g_motor_count);
    /* Progress watchdog: if we're "moving" but angle sum hasn't changed for a while,
       the bus is likely wedged. Recover I2C, re-init PCA, and re-send current angles. */
    {
      static uint32_t last_sum = 0, last_progress_ms = 0;
      uint32_t now = HAL_GetTick();

      // Sum of current angles as a cheap movement-progress proxy
      uint32_t sum = 0;
      for (uint8_t i=0;i<g_motor_count;i++) sum += g_motion[i].current;

      if (s_step_active) {
        if (sum != last_sum) {
          last_sum = sum;
          last_progress_ms = now;
        } else if ((now - last_progress_ms) > STALL_MS) {
          // 1) Bus recover + PCA re-init
          HAL_I2C_DeInit(&hi2c1); HAL_Delay(2); MX_I2C1_Init(); HAL_Delay(1);
          PCA9685_Init(50);

          // 2) Re-send the *current* angles so the chip and our state are in sync
          for (uint8_t i=0;i<g_motor_count;i++) {
            PCA9685_SetServoAngle(g_servo_id[i], g_motion[i].current);
            g_motion[i].next_step_ms = now; // nudge next steps immediately
          }

          // 3) Tell the host (UART1) and your console (UART2)
          send_line(&huart1, "ERR:I2CSTALL\n");
          LOGI("I2C recovered & PCA re-synced");

          last_progress_ms = now; // reset stall timer
        }
      } else {
        last_sum = sum;
        last_progress_ms = now;
      }
    }

    /* 4) Completion notification (DONE) + optional AUTO loop */
    if (s_step_active) {
      if (all_reached(g_motor_count)) {
        uint32_t elapsed_ms = HAL_GetTick() - s_move_start_ms;
        char line[32];
        int n = snprintf(line, sizeof(line), "ELAPSED:%lu\n", (unsigned long)elapsed_ms);
        if (n>0) send_line(&huart1, line);
        send_line(&huart1,"DONE\n");
        s_last_cmd_ms = HAL_GetTick();   // grace time after completion
        s_step_active = 0;

        if (MF_GetMode() == MF_MODE_AUTO && g_have_last_cmd) {
          g_auto_flip ^= 1;
          for (uint8_t i=0;i<g_motor_count;i++){
            uint16_t tgt = g_auto_flip ? SAFE_POSE[i] : g_last_cmd_angles[i];
            g_motion[i].target = tgt;
          }
          s_step_active = 1;
          s_last_cmd_ms = HAL_GetTick();
        }
      }
    }


    /* 5) RX watchdog -> glide to SAFE pose once (disabled in MANUAL mode) */
    uint32_t now = HAL_GetTick();
    uint8_t mode = MF_GetMode();
    if (!s_in_failsafe
        && WATCHDOG_MS
        && (mode == MF_MODE_AUTO)   /* only enforce WD in AUTO */
        && (now - s_last_cmd_ms) > WATCHDOG_MS)
    {
      for (uint8_t i=0;i<g_motor_count;i++){
        g_motion[i].target        = SAFE_POSE[i];
        g_motion[i].step_delay_ms = DEFAULT_STEP_DELAY_MS;
      }
      s_in_failsafe = 1;
      send_line(&huart1,"ERR:WATCHDOG\n");
      s_last_cmd_ms = now;  // avoid RX watchdog during recovery

    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* ────────────────────────── Local helpers ───────────────────────────────── */
static void send_line(UART_HandleTypeDef* huart, const char *s)
{
  HAL_UART_Transmit(huart, (uint8_t*)s, (uint16_t)strlen(s), 100);
}

/* Human-readable mirror on UART2: "F#:angle,speed;..." */
static void send_snapshot_uart2(void)
{
  char line[192];
  int off = snprintf(line, sizeof(line), "SNAP N=%u |", g_motor_count);
  for (uint8_t i = 0; i < g_motor_count && off < (int)sizeof(line)-12; ++i) {
    off += snprintf(line+off, sizeof(line)-off, " %u:%u,%u;", i,
                    (unsigned)g_motion[i].target,
                    (unsigned)g_motion[i].step_delay_ms);
  }
  LOGI("%s", line);
}


/* Initialize motion tables to SAFE pose at default speed */
static void Motion_Init(uint8_t count)
{
	TRACE_IN();

  uint32_t now = HAL_GetTick();

  for (uint8_t i=0;i<count;i++){
    g_speed_latched[i]           = 0;
    g_finger[i].angle            = SAFE_POSE[i];
    g_finger[i].speed_ms_per_deg = g_global_ms_per_deg;

    g_motion[i].current          = SAFE_POSE[i];
    g_motion[i].target           = SAFE_POSE[i];
    g_motion[i].step_delay_ms    = g_global_ms_per_deg;
    g_motion[i].next_step_ms     = now;
    LOGI("INIT %s: cur=%u tgt=%u step=%u latch=%u G=%u",
		 MF_MotorName(i),
         (unsigned)g_motion[i].current,
         (unsigned)g_motion[i].target,
         (unsigned)g_motion[i].step_delay_ms,
         (unsigned)g_speed_latched[i],
         (unsigned)g_global_ms_per_deg);

    PCA9685_SetServoAngle(g_servo_id[i], SAFE_POSE[i]);
    g_last_cmd_angles[i]         = SAFE_POSE[i];
  }
  g_have_last_cmd = 0;
  g_auto_flip = 0;
  TRACE_OUT();
}


/* Simple per-degree stepping (non-blocking, I2C-safe round-robin) */
static void Motion_Update(uint8_t count)
{


  static uint8_t rr = 0;  // round-robin cursor
  uint32_t now = HAL_GetTick();
  uint8_t quota = I2C_WRITES_PER_LOOP;
  uint8_t start = rr;

  for (uint8_t k = 0; k < count && quota; ++k) {
    uint8_t i = (uint8_t)((start + k) % count);

    if (g_motion[i].current == g_motion[i].target) continue;
    if ((int32_t)(now - g_motion[i].next_step_ms) < 0) continue;

    // If I2C isn't ready, back off a hair and try later (don't pile up writes)
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
      g_motion[i].next_step_ms = now + 2;
      break; // give the bus a moment
    }

    if (g_motion[i].current < g_motion[i].target) g_motion[i].current++;
    else                                          g_motion[i].current--;

    PCA9685_SetServoAngle(g_servo_id[i], g_motion[i].current);
    g_motion[i].next_step_ms = now + g_motion[i].step_delay_ms;
    quota--;
  }

  rr = (uint8_t)((start + 1) % count);

  LOGD_EVERY(50, "Motion_Update active; rr=%u", rr);  // once every 50 ms
}



static uint8_t all_reached(uint8_t count)
{
	TRACE_IN();

  for (uint8_t i=0;i<count;i++){
    if (g_motion[i].current != g_motion[i].target) return 0;
  }
  return 1;

  TRACE_OUT();
}

/**
 * @brief Commit A/V updates from parser. Also remembers last A: set
 *        to enable simple AUTO+LOOP ping-pong between SAFE and last A list.
 */
static void apply_targets_from_parser(uint32_t mask, uint8_t count, uint8_t *out_touched_angles)
{
  TRACE_IN();

#if LOG_LEVEL >= LOG_LEVEL_INFO
  LOGI(">> %lu %s()", (unsigned long)HAL_GetTick(), __func__);
#endif

  uint8_t touched = 0;                 // local flag (avoids repeated *out_touched_angles writes)
  uint32_t msk = mask;                 // walk the update mask bit-by-bit

  for (uint8_t i = 0; (i < count) && msk; ++i, msk >>= 1) {
    if ((msk & 1u) == 0u) continue;    // no update for channel i this frame

    // Snapshot old values once (for logs)
    uint16_t old_tgt   = g_motion[i].target;
    uint16_t old_step  = g_motion[i].step_delay_ms;
    uint8_t  old_latch = g_speed_latched[i];

    // ---- Angle (A) ----
    const MF_Motor m = MF_GetState(i);               // small POD; fetched once
    uint16_t tgt = clamp_u16(m.angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    g_finger[i].angle  = tgt;
    g_motion[i].target = tgt;

    if (tgt != old_tgt) {
      touched = 1;
      LOGI("A %s: %u -> %u", MF_MotorName(i), (unsigned)old_tgt, (unsigned)tgt);
    }

    // ---- Speed (V) ----
    // Only grab time if we actually change the schedule.
    if (m.speed_ms_per_deg > 0) {
      // Per-motor latch: V > 0
      uint16_t step_ms = (uint16_t)m.speed_ms_per_deg;
      g_finger[i].speed_ms_per_deg = step_ms;
      g_motion[i].step_delay_ms    = step_ms;
      g_speed_latched[i]           = 1;

      // take effect mid-move
      uint32_t now_ms = HAL_GetTick();
      g_motion[i].next_step_ms     = now_ms;

      if ((old_step != step_ms) || (old_latch != 1)) {
        LOGI("V latch %s: step %u->%u, latch %u->1",
        	MF_MotorName(i), (unsigned)old_step, (unsigned)step_ms, (unsigned)old_latch);
      }
    } else {
      // V == 0 → unlatch and follow current global (if we were latched)
      if (old_latch) {
        g_speed_latched[i]           = 0;
        g_finger[i].speed_ms_per_deg = g_global_ms_per_deg;
        g_motion[i].step_delay_ms    = g_global_ms_per_deg;

        uint32_t now_ms = HAL_GetTick();
        g_motion[i].next_step_ms     = now_ms;

        LOGI("V unlatch % -> follow G=%u",
			MF_MotorName(i), (unsigned)g_global_ms_per_deg);

        if ((old_step != g_motion[i].step_delay_ms) || (old_latch != 0)) {
          LOGI("S %u: step %u->%u, latch %u->0",
			  MF_MotorName(i), (unsigned)old_step, (unsigned)g_motion[i].step_delay_ms, (unsigned)old_latch);
        }
      } else {
        // already following global; keep in sync silently (no extra logs)
        g_finger[i].speed_ms_per_deg = g_global_ms_per_deg;
        g_motion[i].step_delay_ms    = g_global_ms_per_deg;
      }
    }
  }

  // Remember last full A: only when at least one angle changed
  if (touched) {
    for (uint8_t i = 0; i < count; ++i) {
      g_last_cmd_angles[i] = g_motion[i].target;
    }
    g_have_last_cmd = 1;
  }

  if (out_touched_angles) *out_touched_angles = touched;

  TRACE_OUT();

#if LOG_LEVEL >= LOG_LEVEL_INFO
  LOGI("<< %lu %s()", (unsigned long)HAL_GetTick(), __func__);
#endif
}


/**
 * @brief React to meta updates from parser:
 *        - N: motor count
 *        - G: global speed
 *        - MODE: MANUAL/AUTO
 *        - LOOP: 0/1
 *
 * Parser exposes a bitmask MF_GetMetaUpdates() so we only touch what changed.
 */
// Handle protocol meta-updates from the framed parser.
// Applies: N (count), G (global speed), MODE, LOOP
// Emits:   DONE:MODE=..., DONE:LOOP=..., DONE:G=..., DONE:N=...
static void handle_meta_updates(void)
{
	TRACE_IN();
  uint8_t meta = MF_GetMetaUpdates();
  if (!meta) return;

  /* Any meta activity counts as host activity */
  s_last_cmd_ms = HAL_GetTick();
  s_in_failsafe = 0;

  /* 1) Motor count (N) — clamp and (re)initialise motion if it changed */
  if (meta & MF_META_COUNT) {
    uint8_t req = MF_GetMotorCount();
    uint8_t new_count = MIN(MAX_MOTORS, MIN(SERVO_NUM, (req ? req : g_motor_count)));
    if (new_count != g_motor_count) {
      g_motor_count = new_count;
      LOGI("N request=%u, current=%u", (unsigned)req, (unsigned)g_motor_count);
      /* (Re)initialise any channels now addressable (idempotent for existing) */
      for (uint8_t i = 0; i < g_motor_count; ++i) {
        if (i >= SERVO_NUM) break;
        g_servo_id[i] = i;	// Initialize the Servo Instance for PCA9685 module
      }
      Motion_Init(g_motor_count);
    }

    TRACE_OUT();

  }

  /* 2) Global speed (G) — apply uniformly (overrides per-channel), immediate effect */
  if (meta & MF_META_GSPEED) {
    uint16_t gs = MF_GetGlobalSpeed();
    if (gs > 0) {
      uint16_t oldG = g_global_ms_per_deg;
      g_global_ms_per_deg = gs;

      uint32_t now = HAL_GetTick();
      LOGI("G global: %u -> %u ms/deg (override)", (unsigned)oldG, (unsigned)gs);

      for (uint8_t i = 0; i < g_motor_count; ++i) {
        uint16_t old_step  = g_motion[i].step_delay_ms;
        uint8_t  old_latch = g_speed_latched[i];

        g_finger[i].speed_ms_per_deg = gs;
        g_motion[i].step_delay_ms    = gs;
        g_motion[i].next_step_ms     = now;   // immediate effect
        g_speed_latched[i]           = 0;     // ✅ clear latch so G truly overrides

        if (old_step != gs || old_latch != 0) {
          LOGI("G apply %s: step %u->%u, latch %u->0",
			  MF_MotorName(i), (unsigned)old_step, (unsigned)gs, (unsigned)old_latch);
        }
      }
    }
  }


  /* 3) MODE / LOOP: no immediate motion here; the main loop reads getters */

  /* 4) ACKs (tagged lines; won't trigger plain-DONE repeaters) */
  if (meta & MF_META_MODE) {
    const char *m = (MF_GetMode() == MF_MODE_AUTO) ? "AUTO" : "MANUAL";
    char line[24];
    int n = snprintf(line, sizeof(line), "DONE:MODE=%s\n", m);
    if (n > 0) send_line(&huart1, line);
    LOGI("System Mode applied = %s", m);
  }
  if (meta & MF_META_LOOP) {
    char line[24];
    int n = snprintf(line, sizeof(line), "DONE:LOOP=%u\n", (unsigned)MF_GetLoop());
    if (n > 0) send_line(&huart1, line);
    LOGI("LOOP Mode applied = %u", (unsigned)MF_GetLoop());
  }
  if (meta & MF_META_GSPEED) {
    char line[24];
    int n = snprintf(line, sizeof(line), "DONE:G=%u\n", (unsigned)MF_GetGlobalSpeed());
    if (n > 0) send_line(&huart1, line);
    LOGI("G applied = %u", (unsigned)MF_GetGlobalSpeed());
  }
  if (meta & MF_META_COUNT) {
    char line[24];

    int n = snprintf(line, sizeof(line), "DONE:N=%u\n", (unsigned)MF_GetMotorCount());
    if (n > 0) send_line(&huart1, line);
    LOGI("N applied = %u", g_motor_count);
  }

  /* 5) Clear handled bits */
  MF_ClearMetaUpdates(meta);

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	(void)file; (void)line;
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
