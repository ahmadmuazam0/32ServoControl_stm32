/*
 * log_uart.c
 *
 *  Created on: Sep 10, 2025
 *      Author: Muazam
 */
#include "log_uart.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *s_uart = NULL;
static uint8_t  s_buf[LOG_BUF_SZ];
static volatile uint16_t s_head = 0, s_tail = 0;
static volatile uint8_t  s_tx_active = 0;
static uint16_t s_curr_len = 0;
static volatile uint32_t s_dropped_msgs = 0;

static inline uint16_t ring_free(void){
  uint16_t h=s_head,t=s_tail;
  return (h>=t)? ((LOG_BUF_SZ-1)-(h-t)) : (t-h-1);
}
static inline uint16_t ring_count(void){
  uint16_t h=s_head,t=s_tail;
  return (h>=t)? (h-t) : (LOG_BUF_SZ-(t-h));
}
static int ring_push_msg(const uint8_t *p, uint16_t n){
  int ok = 1;
  __disable_irq();
  uint16_t h = s_head, t = s_tail;
  uint16_t free = (h >= t) ? ((LOG_BUF_SZ - 1) - (h - t)) : (t - h - 1);

  if (n > free) {                  // not enough room → DROP NEW (coherent)
    s_dropped_msgs++;
    ok = 0;
  } else {
    while (n--) { s_buf[h] = *p++; h = (uint16_t)((h + 1) % LOG_BUF_SZ); }
    s_head = h;
  }
  __enable_irq();
  return ok;
}
static void kick_tx(void){
  if (!s_uart || s_tx_active) return;

  uint16_t cnt = ring_count();
  if (!cnt) return;

  uint16_t t = s_tail, h = s_head;
  uint16_t chunk = (h > t) ? (h - t) : (LOG_BUF_SZ - t);

  HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(s_uart, &s_buf[t], chunk);
  if (st == HAL_OK) {
    s_curr_len  = chunk;
    s_tx_active = 1;
  }
  // if BUSY/ERROR we’ll retry from LOG_Poll()
}

void LOG_Init(UART_HandleTypeDef *huart2){
  s_uart=huart2; s_head=s_tail=0; s_tx_active=0; s_curr_len=0;
}


void LOG_Poll(void){
  // If TX is "active" but DMA is actually idle (e.g., missed callback),
  // advance tail and allow next chunk
  if (s_tx_active) {
    if (!s_uart->hdmatx || s_uart->hdmatx->State == HAL_DMA_STATE_READY) {
      __disable_irq();
      s_tail = (uint16_t)((s_tail + s_curr_len) % LOG_BUF_SZ);
      s_curr_len  = 0;
      s_tx_active = 0;
      __enable_irq();
    }
  }
  if (!s_tx_active) kick_tx();

  // If messages were dropped and there is space, emit a single notice
  static uint8_t in_notice = 0;
  if (!in_notice) {
    uint32_t dropped = s_dropped_msgs;
    if (dropped) {
      char note[64];
      int n = snprintf(note, sizeof(note), "[LOG] dropped %lu msgs\n", (unsigned long)dropped);
      if (n > 0 && ring_push_msg((uint8_t*)note, (uint16_t)n)) {
        s_dropped_msgs = 0;
        in_notice = 1;   // prevent recursion via kick_tx→callback→LOG_Poll
        kick_tx();
        in_notice = 0;
      }
    }
  }
}


void LOGf(int level, const char *fmt, ...){
  (void)level;
  char tmp[160]; int n=0;
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
  n = snprintf(tmp,sizeof(tmp),"[%8lu] ", (unsigned long)HAL_GetTick());
#endif
  va_list ap; va_start(ap, fmt);
  int m = vsnprintf(tmp+n, (int)sizeof(tmp)-n, fmt, ap);
  va_end(ap);
  if (m < 0) return;
  size_t len = (size_t)n + (size_t)m;
  if (len >= sizeof(tmp)-2) len = sizeof(tmp)-2;
  tmp[len++] = '\n';

  if (!ring_push_msg((uint8_t*)tmp, (uint16_t)len)) {
    // Dropped this message silently; LOG_Poll() will emit a summary.
    return;
  }
  kick_tx();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if (huart != s_uart) return;
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  __disable_irq();
  s_tail = (uint16_t)((s_tail + s_curr_len) % LOG_BUF_SZ);
  s_curr_len = 0; s_tx_active = 0;
  __enable_irq();
  kick_tx();
}
