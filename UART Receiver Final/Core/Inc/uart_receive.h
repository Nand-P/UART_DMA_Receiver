/*
 * uart_receive.h
 *
 *  Created on: February 9, 2024
 *      Author: Nand Patel
 */

#ifndef INC_UART_RECEIVE_H_
#define INC_UART_RECEIVE_H_

#include <uart_common.h>

int receive_gps_data(uint8_t* packets, connection* p_recv_init, data* dest);

#endif /* INC_UART_RECEIVE_H_ */
