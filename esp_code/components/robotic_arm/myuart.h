#ifndef _MYUART_H_
#define _MYUART_H_

#include "stdint.h"

#define  RX_PIN  GPIO_NUM_17
#define  TX_PIN  GPIO_NUM_1

#define  DATA_LENGTH    32

void myuart_init(void);
void uart_senddata(uint8_t *txdata_buf);
int uart_receivedata(uint8_t *rx_buf, int buf_max_len);

#endif
