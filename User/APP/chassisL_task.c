/**
  *********************************************************************
  * @file      chassisL_task.c/h
  * @brief     该任务控制左腿的电机，都是DM4340，这些电机挂载在can2总线上
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "chassisL_task.h"
#include "control_board_profile.h"
#include "fdcan.h"

#include "cmsis_os.h"
#include <stddef.h>

extern chassis_t chassis_move;

uint32_t CHASSL_TIME=1;

float my_kd=0.0f;
float my_vel=0.0f;
float my_kp=0.0f;
float my_pos=0.0f;
int b=0;

void ChassisL_task(void)
{
    const ControlBoardProfile *profile = ControlBoardProfile_GetActive();
    const RobotJointId *joint_order = profile->left_leg_joint_order;
    const size_t joint_count = profile->left_leg_joint_count;

    if (!profile->enable_left_leg_task || joint_count == 0U)
    {
        osDelay(CHASSL_TIME);
        return;
    }

    chassis_move.start_flag=1;
    osDelay(2000);
    ChassisL_init(&chassis_move);

    while(1)
    {
        if(chassis_move.start_flag==1)
        {
            for(size_t i = 0; i < joint_count; ++i)
            {
                RobotJointManager_SendMITUsingCache(joint_order[i]);
            }

        }
        else
        {
            for(size_t i = 0; i < joint_count; ++i)
            {
                RobotJointManager_SendMIT(joint_order[i], my_pos, my_vel, my_kp, my_kd, 0.0f);
            }
        }

        if(b==1)
        {
            for(size_t i = 0; i < joint_count; ++i)
            {
                RobotJointManager_SaveZero(joint_order[i]);
                osDelay(CHASSL_TIME);
            }
        }
        osDelay(CHASSL_TIME);

    }
}

void ChassisL_init(chassis_t *chassis)
{
    const ControlBoardProfile *profile = ControlBoardProfile_GetActive();
    const RobotJointId *joint_order = profile->left_leg_joint_order;
    const size_t joint_count = profile->left_leg_joint_count;

    if (!profile->enable_left_leg_task || joint_count == 0U)
    {
        chassis->start_flag = 0U;
        return;
    }

    for (size_t i = 0; i < joint_count; ++i)
    {
        RobotJointManager_RegisterJoint(joint_order[i], &chassis->joint_motor[i], MIT_MODE);
    }

    for (size_t i = 0; i < profile->left_leg_limb_count; ++i)
    {
        RobotJointManager_EnableLimb(profile->left_leg_limbs[i], 10, 20);
    }
}
