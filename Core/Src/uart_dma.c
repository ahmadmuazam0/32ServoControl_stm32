#include "uart_dma.h"
#include "motor_frame.h"

static UART_HandleTypeDef* s_huart;
static uint8_t             s_rxbuf[U1RX_BUF_SIZE];
static volatile uint16_t   s_head; // software consume cursor

// Compute DMA producer index: RX_BUF_SIZE - CNDTR
static inline uint16_t dma_head(void)
{
    return (uint16_t)(U1RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(s_huart->hdmarx));
}

static void feed_range(uint16_t from, uint16_t to)
{
    // Feed [from .. to) into parser (handles wrap outside)
    if (to > from) {
        MF_ProcessBytes(&s_rxbuf[from], (size_t)(to - from));
    } else if (to < from) {
        MF_ProcessBytes(&s_rxbuf[from], (size_t)(U1RX_BUF_SIZE - from));
        MF_ProcessBytes(&s_rxbuf[0],    (size_t)(to));
    }
}

void U1RX_Init(UART_HandleTypeDef* huart1)
{
    s_huart = huart1;
    s_head  = 0;
}

void U1RX_Start(void)
{
    // Start circular DMA reception
    HAL_UART_Receive_DMA(s_huart, s_rxbuf, U1RX_BUF_SIZE);

    // Enable IDLE line interrupt and clear any pending flag
    __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(s_huart);

    // Init parser state & motor store
    MF_Init();
}

void U1RX_OnUsartIrq(UART_HandleTypeDef* s_huart)
{
    // IDLE detected?
    if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_IDLE) &&
        __HAL_UART_GET_IT_SOURCE(s_huart, UART_IT_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(s_huart); // (reads SR then DR under the hood)

        uint16_t head_now = dma_head();
        feed_range(s_head, head_now);
        s_head = head_now;
    }
}

void U1RX_Poll(void)
{
    uint16_t head_now = dma_head();
    if (head_now != s_head) {
        feed_range(s_head, head_now);
        s_head = head_now;
    }
}

const uint8_t* U1RX_Buffer(void) { return s_rxbuf; }
uint16_t       U1RX_BufferSize(void) { return U1RX_BUF_SIZE; }
