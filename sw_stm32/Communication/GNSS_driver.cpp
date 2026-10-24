/***********************************************************************//**
 * @file		GNSS_driver.cpp
 * @brief		GNSS data reception from a uBlox M9N or F9H satellite receiver
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 		This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "stm32f4xx_hal.h"
#include "GNSS.h"
#include "GNSS_driver.h"
#include "UBX_receiver.h"

#if RUN_GNSS

#define POLL_MS 2	// how often the task looks at the DMA buffer
#define IDLE_FLUSH_MS 20 // a heading message comes in the same burst as its PVT

static_assert( GPS_DMA_buffer_SIZE == UBX_PVT_FRAME_SIZE, "UBX_receiver: NAV-PVT length");
static_assert( GPS_RELPOS_DMA_buffer_SIZE == UBX_NAV_RELPOSNED_LENGTH + UBX_FRAME_OVERHEAD, "UBX_receiver: NAV-RELPOSNED length");
static_assert( GPS_DAHEADING_DMA_buffer_SIZE == UBX_NAV_DAHEADING_LENGTH + UBX_FRAME_OVERHEAD, "UBX_receiver: NAV-DAHEADING length");

COMMON UART_HandleTypeDef huart3;
COMMON DMA_HandleTypeDef hdma_usart3_rx;
COMMON  static TaskHandle_t USART3_task_Id = NULL;
COMMON static UBX_receiver ubx_receiver;

/**
 * @brief USART3 Initialization Function
 */
static inline void MX_USART3_UART_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
      ASSERT(0);

    __HAL_LINKDMA( &huart3, hdmarx, hdma_usart3_rx);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
      ASSERT(0);

    HAL_NVIC_SetPriority (DMA1_Stream1_IRQn, STANDARD_ISR_PRIORITY, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream1_IRQn);
}

/**
 * @brief This function handles DMA2 stream2 global interrupt.
 */
extern "C" void
DMA1_Stream1_IRQHandler (void)
{
  BaseType_t HigherPriorityTaskWoken=0;
  HAL_DMA_IRQHandler (&hdma_usart3_rx);
  vTaskNotifyGiveFromISR( USART3_task_Id, &HigherPriorityTaskWoken);
  portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
}

uint8_t __ALIGNED(USART_3_RX_BUFFER_SIZE_ROUND_UP) USART_3_RX_buffer[USART_3_RX_BUFFER_SIZE];

#if MEASURE_GNSS_REFRESH_TIME
uint64_t getTime_usec(void);
COMMON uint64_t delta,start,gnss_max;
COMMON uint64_t gnss_min=-1;
#endif

// see documentation/gnss_receive_path.md
static void process_epoch (GNSS_configration_t gnss)
{
#if MEASURE_GNSS_REFRESH_TIME
  delta = getTime_usec() - start;
  if( delta >gnss_max)
    gnss_max=delta;
  if( delta < gnss_min)
    gnss_min=delta;
  start = getTime_usec();
#endif

  switch( gnss)
  {
    case GNSS_M9N:
    case GNSS_F9P_F9H:
      GNSS.update (ubx_receiver.epoch ());
      break;
    case GNSS_F9P_F9P:
    case GNSS_X20D:
      GNSS.update_combined (ubx_receiver.epoch (), gnss);
      break;
    default:
      ASSERT( 0);
      break;
  }
}

void USART_3_runnable (void *GNSS_type)
{
  GNSS_configration_t gnss = *(GNSS_configration_t *)GNSS_type;

  switch( gnss)
  {
    case GNSS_M9N:
    case GNSS_F9P_F9H: // the heading comes on UART4, see D_GNSS_driver.cpp
      ubx_receiver.reset (0);
      break;
    case GNSS_F9P_F9P:
      ubx_receiver.reset (UBX_NAV_RELPOSNED_ID);
      break;
    case GNSS_X20D:
      ubx_receiver.reset (UBX_NAV_DAHEADING_ID);
      break;
    default:
      ASSERT( 0);
      break;
  }

  USART3_task_Id = xTaskGetCurrentTaskHandle ();
  MX_USART3_UART_Init ();

  drop_privileges();

  // circular mode: the DMA keeps running, the task follows its write position
  while (HAL_UART_Receive_DMA (&huart3, USART_3_RX_buffer, USART_3_RX_BUFFER_SIZE) != HAL_OK)
    {
      HAL_UART_Abort (&huart3);
      delay (POLL_MS);
    }

  unsigned tail = 0;
  TickType_t last_data = xTaskGetTickCount ();
  bool idle_handled = true;

  while (true)
    {
      uint32_t pulNotificationValue;
      (void) xTaskNotifyWait (0, 0xffffffff, &pulNotificationValue, POLL_MS);

      unsigned head = (USART_3_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER (&hdma_usart3_rx)) % USART_3_RX_BUFFER_SIZE;
      TickType_t now = xTaskGetTickCount ();

      if (head != tail)
	{
	  last_data = now;
	  idle_handled = false;
	}

      while (tail != head)
	{
	  if (ubx_receiver.feed (USART_3_RX_buffer[tail]))
	    process_epoch (gnss);
	  tail = (tail + 1) % USART_3_RX_BUFFER_SIZE;
	}

      if (!idle_handled && (now - last_data >= pdMS_TO_TICKS(IDLE_FLUSH_MS)))
	{
	  if (ubx_receiver.flush ())
	    process_epoch (gnss);
	  idle_handled = true;
	}
    }
}

#endif
