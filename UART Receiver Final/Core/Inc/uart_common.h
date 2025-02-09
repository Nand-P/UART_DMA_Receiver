/*
 * uart_common.h
 *
 *  Created on: Nov 14, 2024
 *      Author: Nand
 */
#include <main.h>
#ifndef INC_UART_COMMON_H_
#define INC_UART_COMMON_H_
# define MAX_PACKET_SIZE 255
# define PACKET_SIZE 3
# define DATA_SIZE 27

typedef struct {
	double x_coord;
	double y_coord;
	double z_coord;

	uint8_t control;
} data;

typedef struct {
    uint8_t packet_size; // Not inclusive of the 1 byte crc
    uint8_t num_of_packets; // Total number of packets to be sent
    uint8_t total_size; // Size of data not including padding (actual size), will be modified as packets are received
    uint8_t crc;

    uint8_t handshake; // true if tx and rx handshake are identical
    uint8_t padding; // Number of padded bytes, equivalent to
    				 // packet_size * num_of_packets - total_size
} connection;

uint8_t calculate_crc(uint8_t* buffer, const size_t data_length);


#endif /* INC_UART_COMMON_H_ */
