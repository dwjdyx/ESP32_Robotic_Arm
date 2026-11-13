#include "myuart.h"
#include "driver/uart.h"
#include "driver/gpio.h"

void myuart_init(void)
{
    uart_config_t uart_cfg = {
          .baud_rate = 9600,
          .data_bits = UART_DATA_8_BITS,
          .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
          .rx_flow_ctrl_thresh = 0,
          .stop_bits = UART_STOP_BITS_1,
          .source_clk = UART_SCLK_DEFAULT,
          .parity = UART_PARITY_DISABLE
    };
    uart_param_config(UART_NUM_1,&uart_cfg);
    uart_set_pin(UART_NUM_1,TX_PIN,RX_PIN,-1,-1);
    uart_driver_install(UART_NUM_1, 1024, 1024, 0, NULL, 0);
}

void uart_senddata(uint8_t *txdata_buf)
{
    uart_write_bytes(UART_NUM_1, txdata_buf, DATA_LENGTH);
}

int uart_receivedata(uint8_t *rx_buf, int buf_max_len)
{
    if(rx_buf == NULL || buf_max_len <= 0) return -1;

    // 最多读不超过缓冲区大小
    int data_len = uart_read_bytes(UART_NUM_1, rx_buf, buf_max_len, pdMS_TO_TICKS(100));
    return data_len;
}
