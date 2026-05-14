/*
 * PCA9685.h
 * Servos @ 50 Hz on PCA9685 (STM32 HAL, I2C1)
 */

#ifndef INC_PCA9685_H_
#define INC_PCA9685_H_

#include "main.h"

/* ── Servo endpoints (us) @ 50 Hz ───────────────────────────────────────── */
#ifndef PCA_SERVO_MIN_US
#define PCA_SERVO_MIN_US   500   // try 1000 if your servos chatter near 0°
#endif
#ifndef PCA_SERVO_MAX_US
#define PCA_SERVO_MAX_US  2000   // try 2000 to limit travel
#endif

/* ── I2C address ─────────────────────────────────────────────────────────── */
#define PCA9685_ADDR_7BIT   0x40           // A5..A0 = 0
#define PCA9685_ADDRESS     (PCA9685_ADDR_7BIT << 1)  // HAL expects 8-bit
// MODE2 OUTDRV bit exists

/* ── Registers (datasheet) ───────────────────────────────────────────────── */
#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_LED0_ON_L   0x06
#define PCA9685_PRE_SCALE   0xFE

/* MODE1 bits */
#define PCA9685_MODE1_AI_BIT       5   // Auto-Increment
#define PCA9685_MODE1_SLEEP_BIT    4
#define PCA9685_MODE1_RESTART_BIT  7

/* MODE2 bits */
#define PCA9685_MODE2_OUTDRV_BIT   2   // 1=totem-pole, 0=open-drain
#define PCA9685_MODE2_INVRT_BIT    4   // 1=inverted outputs

/* ── API ─────────────────────────────────────────────────────────────────── */
void PCA9685_SetBit(uint8_t reg, uint8_t bit, uint8_t value);
void PCA9685_SetPWMFrequency(uint16_t frequency);
void PCA9685_Init(uint16_t frequency);
void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
void PCA9685_SetServoAngle(uint8_t Channel, float Angle);

#endif /* INC_PCA9685_H_ */
