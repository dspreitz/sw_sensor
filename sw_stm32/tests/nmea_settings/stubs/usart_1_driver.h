// Host test build: UART1 reception is fed by the test (nmea_settings_test.cpp).
#ifndef USART_1_DRIVER_H_
#define USART_1_DRIVER_H_
#include <stdint.h>
bool UART1_Receive( uint8_t *pRxByte, uint32_t timeout);
#endif
