/*
 * uart_dma.h
 *
 *  Created on: Aug 18, 2025
 *      Author: Lenovo
 */

#ifndef INC_UART_DMA_H_
#define INC_UART_DMA_H_

#pragma once
#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// Configure your RX ring size here (power of two is convenient but not required)
#ifndef U1RX_BUF_SIZE
#define U1RX_BUF_SIZE 512u
#endif

// Initialize with a configured USART1 handle (from CubeMX).
void U1RX_Init(UART_HandleTypeDef* huart1);

// Start circular RX DMA + enable IDLE interrupt.
void U1RX_Start(void);

// Call from USART1_IRQHandler() to handle IDLE and feed new bytes to the parser.
// IMPORTANT: call this before or after HAL_UART_IRQHandler(&huart1) — both are fine,
// as we clear IDLE explicitly inside.
void U1RX_OnUsartIrq(UART_HandleTypeDef* s_huart);

// Optional polling alternative (call in main loop if you prefer not to parse in IRQ).
void U1RX_Poll(void);

// Expose the raw RX buffer (read-only) if needed for debugging.
const uint8_t* U1RX_Buffer(void);
uint16_t       U1RX_BufferSize(void);

#ifdef __cplusplus
}
#endif




#endif /* INC_UART_DMA_H_ */
