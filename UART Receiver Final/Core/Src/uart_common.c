/*
 * uart_common.c
 *
 *  Created on: Nov 29, 2024
 *      Author: Rahul
 */
#include <uart_common.h>

uint8_t calculate_crc(uint8_t* buffer, const size_t data_length) {
    /*
     * This function calculates an 8-bit CRC checksum given a buffer
     * of uint8_t data using the HAL_CRC_Calculate API.
     *
     * MX_CRC_Init() must be created from the .ioc setup and configured
     * to output an 8-bit CRC.
     *
     * If the buffer is terminated with an 8-bit checksum, it must be
     * excluded from this function by passing sizeof(buffer) - 1 for the
     * data_length parameter. This will prevent it from being included
     * as part of the checksum calculation.
     *
     * INPUTS
     *         buffer : const uint8_t *
     *         Buffer to calculate CRC from
     *
     *         data_length : const size_t
     *         Size of data in bytes. Equivalent to the number of elements
     *         corresponding to data in the buffer.
     *
     * OUTPUTS
     *         crc : uint8_t
     *         8-bit CRC checksum
    */

    uint8_t crc = 0;
    uint8_t *byte = buffer;

    // Sum all bytes of data
    for (int i=0; i < data_length; i++) {
        crc += *byte;
        byte++;
    }

    // Take 2s complement of the sum
    crc = (~crc) + 1;

    return crc;
}


