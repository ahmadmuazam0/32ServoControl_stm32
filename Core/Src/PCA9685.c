/*
 * PCA9685.c  — robust I2C version (retry + recover)
 * Works around HAL BUSY/ERR/TO after bursts of writes (typical with servos).
 */

#include "PCA9685.h"
#include "main.h"
#include "i2c.h"
// #include "usart.h" // (optional) for debug prints

// ── Tunables ────────────────────────────────────────────────────────────────
#define I2C_TO_MS       50    // a bit generous; avoids false timeouts on noise
#ifndef PCA_SERVO_MIN_US
#define PCA_SERVO_MIN_US 500  // you already have these in the header; keep ifdefs
#endif
#ifndef PCA_SERVO_MAX_US
#define PCA_SERVO_MAX_US 2500
#endif

// ── I2C helpers: retry once with auto-reinit ────────────────────────────────
static HAL_StatusTypeDef I2C1_Recover(void) {
  HAL_I2C_DeInit(&hi2c1);
  HAL_Delay(2);
  MX_I2C1_Init();
  HAL_Delay(1);
  return HAL_OK;
}
static void PCA9685_SoftReset(void) {
  uint8_t cmd = 0x06;           // SWRST (per PCA9685 datasheet)
  HAL_I2C_Master_Transmit(&hi2c1, 0x00, &cmd, 1, 10);  // General Call address
  HAL_Delay(1);
}

static HAL_StatusTypeDef PCA_I2C_Read(uint8_t reg, uint8_t *buf, uint16_t len) {
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, reg, 1, buf, len, I2C_TO_MS);
  if (st == HAL_OK) return HAL_OK;
  I2C1_Recover();
  return HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, reg, 1, buf, len, I2C_TO_MS);
}

static HAL_StatusTypeDef PCA_I2C_Write(uint8_t reg, const uint8_t *buf, uint16_t len) {
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, reg, 1, (uint8_t*)buf, len, I2C_TO_MS);
  if (st == HAL_OK) return HAL_OK;

  PCA9685_SoftReset();
  I2C1_Recover();
  return HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, reg, 1, (uint8_t*)buf, len, I2C_TO_MS);
}

// ── Register bit helper ─────────────────────────────────────────────────────
void PCA9685_SetBit(uint8_t reg, uint8_t bit, uint8_t value)
{
  uint8_t v = 0;
  (void)PCA_I2C_Read(reg, &v, 1);                  // if read fails, v stays 0
  if (value) v |=  (1u << bit);
  else        v &= ~(1u << bit);
  (void)PCA_I2C_Write(reg, &v, 1);
  HAL_Delay(1);                                     // small settle
}

// ── Frequency/prescaler ─────────────────────────────────────────────────────
void PCA9685_SetPWMFrequency(uint16_t frequency)
{
  if (frequency < 24)   frequency = 24;
  if (frequency > 1526) frequency = 1526;

  // Datasheet: prescale = round(25MHz/(4096*freq) - 1)
  float prescale_f = (25000000.0f / (4096.0f * (float)frequency)) - 1.0f;
  uint8_t prescale = (uint8_t)(prescale_f + 0.5f);

  // Sleep -> write prescale -> wake -> restart
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
  (void)PCA_I2C_Write(PCA9685_PRE_SCALE, &prescale, 1);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
  HAL_Delay(1);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

// ── Init ────────────────────────────────────────────────────────────────────
void PCA9685_Init(uint16_t frequency)
{
  // MODE2: OUTDRV=1 (totem pole), INVRT=0
  uint8_t mode2 = (1u << PCA9685_MODE2_OUTDRV_BIT);
  (void)PCA_I2C_Write(PCA9685_MODE2, &mode2, 1);

  PCA9685_SetPWMFrequency(frequency); // 50 Hz for hobby servos

  // MODE1: enable Auto-Increment for multi-byte writes
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}

// ── Set PWM for a channel ───────────────────────────────────────────────────
void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
  if (Channel > 15) return;
  uint8_t reg = PCA9685_LED0_ON_L + (4u * Channel);
  uint8_t pwm[4];
  pwm[0] = (uint8_t)(OnTime  & 0xFF);
  pwm[1] = (uint8_t)(OnTime  >> 8);
  pwm[2] = (uint8_t)(OffTime & 0xFF);
  pwm[3] = (uint8_t)(OffTime >> 8);
  (void)PCA_I2C_Write(reg, pwm, 4);
}

// ── Angle helper ────────────────────────────────────────────────────────────
void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
  if (Angle < 0.f)   Angle = 0.f;
  if (Angle > 180.f) Angle = 180.f;

  // 20,000 us period @ 50Hz -> 4096 counts
  float us = PCA_SERVO_MIN_US + (Angle / 180.0f) * (PCA_SERVO_MAX_US - PCA_SERVO_MIN_US);
  uint16_t counts = (uint16_t)((us * 4096.0f / 20000.0f) + 0.5f);
  PCA9685_SetPWM(Channel, 0, counts);
}
