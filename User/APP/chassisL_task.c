/**
  *********************************************************************
  * @file      chassisL_task.c/h
  * @brief     该任务控制左腿的五个电机，都是DM4340，这五个电机挂载在can2总线上
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

static const RobotJointId left_leg_joint_order[] = {
    ROBOT_JOINT_LEFT_LEG_HIP_PITCH,
    ROBOT_JOINT_LEFT_LEG_HIP_YAW,
    ROBOT_JOINT_LEFT_LEG_HIP_ROLL,
    ROBOT_JOINT_LEFT_LEG_KNEE,
    ROBOT_JOINT_LEFT_LEG_ANKLE_PITCH,
    ROBOT_JOINT_LEFT_LEG_ANKLE_ROLL,
    ROBOT_JOINT_LEFT_LEG_TOE,
};

#define LEFT_LEG_JOINT_COUNT (sizeof(left_leg_joint_order) / sizeof(left_leg_joint_order[0]))
void ChassisL_task(void)
{
	chassis_move.start_flag=1;
	osDelay(2000);
    ChassisL_init(&chassis_move);

        while(1)
        {
                 if(chassis_move.start_flag==1)
                 {
                        for(size_t i = 0; i < LEFT_LEG_JOINT_COUNT; ++i)
                        {
                                RobotJointManager_SendMITUsingCache(left_leg_joint_order[i]);
                        }

                 }
                 else
                 {
                        RobotJointManager_SendMIT(ROBOT_JOINT_LEFT_LEG_HIP_PITCH, my_pos,my_vel,my_kp, my_kd,0.0f);//left_pitch
                        RobotJointManager_SendMIT(ROBOT_JOINT_LEFT_LEG_HIP_YAW, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left_yaw
                        RobotJointManager_SendMIT(ROBOT_JOINT_LEFT_LEG_HIP_ROLL, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left_roll
                        RobotJointManager_SendMIT(ROBOT_JOINT_LEFT_LEG_KNEE, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left_calf
                        RobotJointManager_SendMIT(ROBOT_JOINT_LEFT_LEG_ANKLE_PITCH, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left_foot
                        RobotJointManager_SendMIT(ROBOT_JOINT_LEFT_LEG_ANKLE_ROLL, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left_foot
                        RobotJointManager_SendMIT(ROBOT_JOINT_LEFT_LEG_TOE, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left_foot
                 }

                if(b==1)
                {
                        RobotJointManager_SaveZero(ROBOT_JOINT_LEFT_LEG_KNEE);
                        osDelay(CHASSL_TIME);
                }
                 osDelay(CHASSL_TIME);

        }
}

void ChassisL_init(chassis_t *chassis)
{
        RobotJointManager_RegisterJoint(ROBOT_JOINT_LEFT_LEG_HIP_PITCH, &chassis->joint_motor[0], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_LEFT_LEG_HIP_YAW, &chassis->joint_motor[1], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_LEFT_LEG_HIP_ROLL, &chassis->joint_motor[2], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_LEFT_LEG_KNEE, &chassis->joint_motor[3], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_LEFT_LEG_ANKLE_PITCH, &chassis->joint_motor[4], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_LEFT_LEG_ANKLE_ROLL, &chassis->joint_motor[5], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_LEFT_LEG_TOE, &chassis->joint_motor[6], MIT_MODE);

        RobotJointManager_EnableLimb(ROBOT_LIMB_LEFT_LEG, 10, 20);
}


