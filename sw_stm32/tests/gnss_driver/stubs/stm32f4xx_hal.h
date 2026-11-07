// Host test build: the HAL types and calls used by GNSS_driver.cpp. The UART
// receive DMA is simulated in sim.cpp.
#ifndef STM32F4XX_HAL_H_
#define STM32F4XX_HAL_H_
#include <stddef.h>
#include <stdint.h>
#include "sim.h"

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct
{
  uint32_t Channel, Direction, PeriphInc, MemInc, PeriphDataAlignment,
      MemDataAlignment, Mode, Priority, FIFOMode;
} DMA_InitTypeDef;
typedef struct { void *Instance; DMA_InitTypeDef Init; void *Parent; } DMA_HandleTypeDef;
typedef struct
{
  uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling;
} UART_InitTypeDef;
typedef struct { void *Instance; UART_InitTypeDef Init; DMA_HandleTypeDef *hdmarx; } UART_HandleTypeDef;

#define USART3 ((void *) 3)
#define DMA1_Stream1 ((void *) 11)
#define GPIOD ((void *) 4)
#define GPIO_PIN_8 0x100
#define GPIO_PIN_9 0x200
#define GPIO_MODE_AF_PP 0
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_VERY_HIGH 0
#define GPIO_AF7_USART3 0
#define DMA_CHANNEL_4 0
#define DMA_PERIPH_TO_MEMORY 0
#define DMA_PINC_DISABLE 0
#define DMA_MINC_ENABLE 0
#define DMA_PDATAALIGN_BYTE 0
#define DMA_MDATAALIGN_BYTE 0
#define DMA_NORMAL 0
#define DMA_CIRCULAR 0x100
#define DMA_PRIORITY_MEDIUM 0
#define DMA_FIFOMODE_DISABLE 0
#define UART_WORDLENGTH_8B 0
#define UART_STOPBITS_1 0
#define UART_PARITY_NONE 0
#define UART_MODE_TX_RX 0
#define UART_HWCONTROL_NONE 0
#define UART_OVERSAMPLING_16 0
#define DMA1_Stream1_IRQn 0
#define USART3_IRQn 0

#define __HAL_RCC_USART3_CLK_ENABLE() do {} while (0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() do {} while (0)
#define __HAL_LINKDMA(h, field, dma) do { (h)->field = &(dma); (dma).Parent = (h); } while (0)
#define __HAL_DMA_GET_COUNTER(h) sim_dma_remaining ()
#define __ALIGNED(x) __attribute__((aligned(x)))

inline void HAL_GPIO_Init (void *, GPIO_InitTypeDef *) {}
inline HAL_StatusTypeDef HAL_DMA_Init (DMA_HandleTypeDef *) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_UART_Init (UART_HandleTypeDef *) { return HAL_OK; }
inline void HAL_NVIC_SetPriority (int, uint32_t, uint32_t) {}
inline void HAL_NVIC_EnableIRQ (int) {}
inline void HAL_DMA_IRQHandler (DMA_HandleTypeDef *) {}
inline void HAL_UART_IRQHandler (UART_HandleTypeDef *) {}
inline HAL_StatusTypeDef HAL_UART_Transmit_IT (UART_HandleTypeDef *, uint8_t *, uint16_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_UART_Receive_DMA (UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
  bool circular = huart->hdmarx && (huart->hdmarx->Init.Mode == DMA_CIRCULAR);
  return sim_dma_start (buffer, size, circular) ? HAL_OK : HAL_BUSY;
}
inline HAL_StatusTypeDef HAL_UART_Abort (UART_HandleTypeDef *) { sim_dma_stop (); return HAL_OK; }
#endif
