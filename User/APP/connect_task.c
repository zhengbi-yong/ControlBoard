/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     Stream joint telemetry to the host computer and forward it over USB.
  * @note
  * @history
  *
  @verbatim
  ===============================================================================

  ===============================================================================
  @endverbatim
  *********************************************************************
  */

#include "connect_task.h"

#include "cmsis_os.h"
#include "bsp_usart1.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "control_board_profile.h"
#include "dm4310_drv.h"
#include <string.h>

extern send_data_t send_data;

uint32_t OBSERVE_TIME=1;// Period of the telemetry task expressed in milliseconds.

static size_t ConnectTask_PrepareFrame(uint8_t *buffer, size_t capacity)
{
    const ControlBoardProfile *profile = ControlBoardProfile_GetActive();
    const size_t max_motors = (capacity > 1U) ? (capacity - 1U) / 5U : 0U;
    size_t joint_count = profile->telemetry_joint_count;

    if (joint_count > max_motors)
    {
        joint_count = max_motors;
    }

    size_t write_index = 0U;
    buffer[write_index++] = FRAME_HEADER;

    for (size_t i = 0; i < joint_count; ++i)
    {
        const RobotJointId joint_id = profile->telemetry_joints[i];
        Joint_Motor_t *motor = RobotJointManager_GetMotor(joint_id);

        const int16_t p = (motor != NULL) ? motor->para.p_int : 0;
        const int16_t v = (motor != NULL) ? motor->para.v_int : 0;
        const int16_t t = (motor != NULL) ? motor->para.t_int : 0;

        buffer[write_index++] = (uint8_t)(p >> 8);
        buffer[write_index++] = (uint8_t)(p & 0xFF);
        buffer[write_index++] = (uint8_t)(v >> 4);
        buffer[write_index++] = (uint8_t)(((v & 0x0F) << 4) | ((t >> 8) & 0x0F));
        buffer[write_index++] = (uint8_t)(t & 0xFF);
    }

    if (write_index < capacity)
    {
        memset(&buffer[write_index], 0, capacity - write_index);
    }

    const size_t payload_length = joint_count * 5U + 1U;
    if (payload_length < capacity)
    {
        buffer[payload_length] = Check_Sum(payload_length, buffer);
    }
    return payload_length + 1U;
}

void    Connect_task(void)
{
  while(1)
        {
                const size_t frame_length = ConnectTask_PrepareFrame(send_data.tx, sizeof(send_data.tx));
                CDC_Transmit_HS((uint8_t *)send_data.tx, frame_length);

          osDelay(OBSERVE_TIME);
        }
}
