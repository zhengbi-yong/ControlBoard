/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右腿的电机，都是DM4340，这些电机挂载在can1总线上
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "chassisR_task.h"
#include "control_board_profile.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include <stddef.h>

chassis_t chassis_move;

uint32_t CHASSR_TIME=1;

float my_kd2=0.0f;
float my_vel2=0.0f;
float my_kp2=0.0f;
float my_pos2=0.0f;
float my_tor2=0.0f;
int a=0;

void ChassisR_task(void)
{
    const ControlBoardProfile *profile = ControlBoardProfile_GetActive();
    const RobotJointId *joint_order = profile->right_leg_joint_order;
    const size_t joint_count = profile->right_leg_joint_count;

    if (!profile->enable_right_leg_task || joint_count == 0U)
    {
        osDelay(CHASSR_TIME);
        return;
    }

    chassis_move.start_flag=1;
    osDelay(2000);
    ChassisR_init(&chassis_move);
    chassis_move.start_flag=1;
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
                RobotJointManager_SendMIT(joint_order[i], my_pos2, my_vel2, my_kp2, my_kd2, my_tor2);
            }
        }
        if(a==1)
        {
            for(size_t i = 0; i < joint_count; ++i)
            {
                RobotJointManager_SaveZero(joint_order[i]);
                osDelay(CHASSR_TIME);
            }
        }
        osDelay(CHASSR_TIME);
    }
}

void ChassisR_init(chassis_t *chassis)
{
    const ControlBoardProfile *profile = ControlBoardProfile_GetActive();
    const RobotJointId *joint_order = profile->right_leg_joint_order;
    const size_t joint_count = profile->right_leg_joint_count;

    if (!profile->enable_right_leg_task || joint_count == 0U)
    {
        chassis->start_flag = 0U;
        return;
    }

    for (size_t i = 0; i < joint_count; ++i)
    {
        RobotJointManager_RegisterJoint(joint_order[i], &chassis->joint_motor[i], MIT_MODE);
    }

    for (size_t i = 0; i < profile->right_leg_limb_count; ++i)
    {
        RobotJointManager_EnableLimb(profile->right_leg_limbs[i], 10, 100);
    }
}

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}
