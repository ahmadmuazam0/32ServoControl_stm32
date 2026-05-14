/*
 * log_uart.h
 *
 *  Created on: Sep 10, 2025
 *      Author: Muazam
 */

#ifndef INC_LOG_UART_H_
#define INC_LOG_UART_H_
#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdarg.h>

#define LOG_LEVEL_NONE  0
#define LOG_LEVEL_ERR   1
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_INFO  3
#define LOG_LEVEL_DEBUG 4
#define LOG_LEVEL LOG_LEVEL_INFO
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

#ifndef LOG_BUF_SZ
#define LOG_BUF_SZ 1024
#endif

void LOG_Init(UART_HandleTypeDef *huart2);
void LOG_Poll(void);
void LOGf(int level, const char *fmt, ...);

#if LOG_LEVEL >= LOG_LEVEL_ERR
  #define LOGE(...) LOGf(LOG_LEVEL_ERR,  __VA_ARGS__)
#else
  #define LOGE(...) ((void)0)
#endif
#if LOG_LEVEL >= LOG_LEVEL_WARN
  #define LOGW(...) LOGf(LOG_LEVEL_WARN, __VA_ARGS__)
#else
  #define LOGW(...) ((void)0)
#endif
#if LOG_LEVEL >= LOG_LEVEL_INFO
  #define LOGI(...) LOGf(LOG_LEVEL_INFO, __VA_ARGS__)
#else
  #define LOGI(...) ((void)0)
#endif
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
  #define LOGD(...) LOGf(LOG_LEVEL_DEBUG, __VA_ARGS__)
  #define TRACE_IN()  LOGD(">> %lu %s()", (unsigned long)HAL_GetTick(), __func__)
  #define TRACE_OUT() LOGD("<< %lu %s()", (unsigned long)HAL_GetTick(), __func__)
#else
  #define LOGD(...) ((void)0)
  #define TRACE_IN()  ((void)0)
  #define TRACE_OUT() ((void)0)
#endif
// Throttled logs (per call-site)
// Throttled DEBUG logging (use only in very hot loops if needed)
#define LOGD_EVERY(ms, ...) do { static uint32_t _t_; uint32_t _n_=HAL_GetTick(); \
  if ((int32_t)(_n_ - _t_) >= (int32_t)(ms)) { _t_=_n_; LOGD(__VA_ARGS__); } } while(0)


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_LOG_UART_H_ */
