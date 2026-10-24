/**
 @file usart3_driver.h
 @brief GNSS USART driver
 @author: Dr. Klaus Schaefer
 */

#define GPS_DMA_buffer_SIZE (sizeof( uBlox_pvt) + 8) // plus "u B class id size1 size2 ... cks1 cks2"
#define GPS_RELPOS_DMA_buffer_SIZE (sizeof( uBlox_relpos_NED) + 8) // plus "u B class id size1 size2 ... cks1 cks2"
#define GPS_DAHEADING_DMA_buffer_SIZE (sizeof( uBlox_DAHEADING) + 8) // plus "u B class id size1 size2 ... cks1 cks2"
// circular DMA buffer: 44 ms at 115200 baud, a power of two for its MPU region
#define USART_3_RX_BUFFER_SIZE 512
#define USART_3_RX_BUFFER_SIZE_ROUND_UP USART_3_RX_BUFFER_SIZE

extern uint8_t USART_3_RX_buffer[];

void USART_3_runnable (void* using_DGNSS);
