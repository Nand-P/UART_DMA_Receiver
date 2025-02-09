/*
 * uart_receive.c
 *
 *  Created on: February 9, 2025
 *      Author: Nand Patel
 */

#include <uart_receive.h>

int receive_gps_data(uint8_t* packets, connection* p_recv_init, data* dest){
	/* Extracts GPS and control data from array of packets to save into a destination data
	 * struct.
	 *
	 *
	 * INPUTS
	 * 		packets : uint8_t*
	 * 		Pointer to array of packets
	 *
	 * 		p_recv_init : const connection*
	 * 		Pointer to connection struct. Useful for determining number of packets,
	 * 		packet size, and number of valid data bytes not including padding.
	 *
	 * 		dest : data*
	 * 		Pointer to data struct
	 *
	 * OUTPUTS
	 *		int
	 *		0 means success, 1 means error
	 * */

	// Ensure that size of destination data struct matches size of received data
	if (sizeof(*dest) != (p_recv_init->packet_size * p_recv_init->num_of_packets) - p_recv_init->padding) {
		return 1;
	}

    uint8_t* curr = dest;

    // For all packets excluding last (due to padding on last packet), save to data struct
    for (int i = 0; i < p_recv_init->num_of_packets - 2; i++) {
    	// Exclude 1 byte CRC from each packet
    	memcpy(curr, &packets[i * (PACKET_SIZE + 1)], p_recv_init->packet_size);
    	curr += p_recv_init->packet_size;
    }

    // Save last packet accounting for any padding
    memcpy(
    	curr,
		&packets[(p_recv_init->num_of_packets - 1) * (PACKET_SIZE + 1)],
		p_recv_init->packet_size - p_recv_init->padding
	);
    curr += p_recv_init->packet_size - p_recv_init->padding; // At this point, curr should be at the end of the data struct

    return 0;
}


