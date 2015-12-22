/**************************************************************************
 * Copyright (c) 2015 - Gray Cat Labs - https://graycat.io
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 **************************************************************************/

/**
 * @file cp2110.h
 * @author Alex Hiam - <alex@graycat.io>
 *
 * @brief A basic userspcae driver for the CP2110 HID USB-UART IC.
 *
 */

#ifndef _CP2110_H_
#define _CP2110_H_

#include <stdint.h>
#include <hidapi/hidapi.h>

#define CP2110_VID 0x10c4
#define CP2110_PID 0xea80

#define REPORT_DATA     0x01
#define REPORT_DATA_MAX 0x3F

#define REPORT_GET_SET_UART_ENABLE 0x41
#define REPORT_SET_PURGE_FIFOS     0x43
#define REPORT_GET_SET_UART_CONFIG 0x50

/**
 * A connected CP2110 device handle.
 */
typedef hid_device CP2110_dev;

/**
 * @brief Enumerates all connected CP2110 devices (with default VID/PID).
 *
 * @return Returns a pointer to an array hidapi hid_device_info structs.
 */
struct hid_device_info *CP2110_enumerate(void);

/**
 * @brief Connects to the first attached CP2110.
 *
 * @return Returns a CP2110_dev pointer to the open CP2110.
 */
CP2110_dev *CP2110_init(void);

/**
 * @brief Closes from the given CP2110 connection.
 *
 * @param handle CP2110_dev pointer of the connected CP2110.
 */
void CP2110_release(CP2110_dev *handle);

/**
 * @brief Checks if the given CP2110 UART is enabled.
 *
 * @param handle CP2110_dev pointer of the connected CP2110.
 *
 * @return Returns 1 if the UART is enabled, 0 if not, -1 if error.
 */
int CP2110_uartEnabled(CP2110_dev *handle);

/**
 * @brief Enables the given CP2110's UART.
 *
 * @param handle CP2110_dev pointer of the connected CP2110.
 *
 * @return Returns 1 if successful, -1 if error.
 */
int CP2110_enableUART(CP2110_dev *handle);

/**
 * @brief Disables the given CP2110's UART.
 *
 * @param handle CP2110_dev pointer of the connected CP2110.
 *
 * @return Returns 1 if successful, -1 if error.
 */
int CP2110_disableUART(CP2110_dev *handle);

/**
 * Passed to #CP2110_purgeFIFO to specify which FIFO(s) to purge.
 */
typedef enum CP2110_fifo {
  FIFO_TX = 1,
  FIFO_RX,
  FIFO_BOTH
} CP2110_fifo;

/**
 * @brief Purges one or both of the given CP2110's FIFOs.
 *
 * @param handle CP2110_dev pointer of the connected CP2110.
 * @param fifo FIFO_TX, FIFO_RX or FIFO_BOTH.
 *
 * @return Returns 1 if successful, -1 if error.
 */
int CP2110_purgeFIFO(CP2110_dev *handle, CP2110_fifo fifo);

/**
 * Passed to #CP2110_setUARTConfig to configure the parity bit.
 */
typedef enum CP2110_parity {
  PARITY_NONE,
  PARITY_EVEN,
  PARITY_ODD,
  PARITY_MARK,
  PARITY_SPACE
} CP2110_parity;

/**
 * Passed to #CP2110_setUARTConfig to configure the flow control.
 */
typedef enum cp2110_flow_control {
  FLOW_CONTROL_DISABLED,
  FLOW_CONTROL_ENABLED
} CP2110_flow_control;

/**
 * Passed to #CP2110_setUARTConfig to set the number of data bits.
 */
typedef enum cp2110_data_bits {
  DATA_BITS_5 = 0x05,
  DATA_BITS_6,
  DATA_BITS_7,
  DATA_BITS_8
} CP2110_data_bits;

/**
 * Passed to #CP2110_setUARTConfig to set the number of stop bits.
 */
typedef enum cp2110_stop_bits {
  STOP_BITS_SHORT,  /**< 1 stop bit. */
  STOP_BITS_LONG    /**< 1.5 stop bits if data bits = 5, else 2 stop bits. */
} CP2110_stop_bits;

/**
 * @brief Configures the given CP2110's UART parameters.
 *
 * @param handle CP2110_dev pointer of the connected CP2110.
 * @param baud the desired baud rate in bps (300 <= \a baud <= 500000).
 * @param parity the desired parity, @see ::CP2110_parity.
 * @param flow_control the desired flow control, @see ::CP2110_flow_control.
 * @param data_bits the desired number of data bits, @see ::CP2110_data_bits.
 * @param stop_bits the desired number of stop bits, @see ::CP2110_stop_bits.
 *
 * @return Returns 1 if successful, -1 if error.
 */
int CP2110_setUARTConfig(CP2110_dev *handle, 
                         uint32_t baud,
                         CP2110_parity parity,
                         CP2110_flow_control flow_control,
                         CP2110_data_bits data_bits,
                         CP2110_stop_bits stop_bits);



int CP2110_getUARTConfig(CP2110_dev *handle, uint8_t *config);

/**
 * @brief Writes \a len bytes from \a tx_buf to the given CP2110 handle.
 *
 * @param handle CP2110_dev pointer of the connected CP2110.
 * @param tx_buf a pointer to the array of bytes to be written.
 * @param len the number of bytes to write.
 *
 * @return Returns the number of bytes written or -1 if error.
 */
int CP2110_write(CP2110_dev *handle, char *tx_buf, int len);

/**
 * @brief Reads \a len bytes to \a rx_buf from the given CP2110 handle.
 *
 * @param handle CP2110_dev pointer of the connected CP2110.
 * @param rx_buf pointer to an initialized array to write the received bytes to.
 * @param len the number of bytes to read.
 *
 * @return Returns the number of bytes read or -1 if error.
 */
int CP2110_read(CP2110_dev *handle, char *rx_buf, int len);

#endif