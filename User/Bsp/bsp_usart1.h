#ifndef VISION_UART_H
#define VISION_UART_H
#include "main.h"


#define SEND_DATA_CHECK   1U          /**< Flag indicating that outbound frames carry a checksum. */
#define READ_DATA_CHECK   0U          /**< Flag indicating that inbound frames require checksum validation. */
#define FRAME_HEADER      0X7B        /**< Start byte for every UART frame. */
#define FRAME_TAIL        0X7D        /**< End byte for every UART frame. */
#define RECEIVE_DATA_SIZE 11U         /**< Number of bytes received from the companion computer.
                                       *< When bit 7 of byte[1] is set the payload encodes a
                                       *< MIT command frame (see tools/motor_test.py). */
#define SEND_DATA_SIZE    72U         /**< Number of bytes transmitted to the companion computer. */

/**
 * @brief Buffer used to build frames sent to the ROS controller.
 */
typedef struct
{
  uint8_t tx[SEND_DATA_SIZE];

}send_data_t;

/**
 * @brief Buffer used to store frames received from the ROS controller.
 */
typedef struct
{
  uint8_t rx[RECEIVE_DATA_SIZE];

}rev_data_t;


extern void connect_usart1_init(void);

extern uint8_t Check_Sum(uint8_t Count_Number,uint8_t *buffer);

#endif


