/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右腿的五个电机，都是DM4340，这五个电机挂载在can1总线上
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

static const RobotJointId right_leg_joint_order[] = {
    ROBOT_JOINT_RIGHT_LEG_HIP_PITCH,
    ROBOT_JOINT_RIGHT_LEG_HIP_ROLL,
    ROBOT_JOINT_RIGHT_LEG_HIP_YAW,
    ROBOT_JOINT_RIGHT_LEG_KNEE,
    ROBOT_JOINT_RIGHT_LEG_ANKLE_PITCH,
    ROBOT_JOINT_RIGHT_LEG_ANKLE_ROLL,
    ROBOT_JOINT_RIGHT_LEG_TOE,
};

#define RIGHT_LEG_JOINT_COUNT (sizeof(right_leg_joint_order) / sizeof(right_leg_joint_order[0]))
void ChassisR_task(void)
{
	chassis_move.start_flag=1;
	osDelay(2000);
    ChassisR_init(&chassis_move);
  chassis_move.start_flag=1;
        while(1)
        {
                if(chassis_move.start_flag==1)
                {
                        for(size_t i = 0; i < RIGHT_LEG_JOINT_COUNT; ++i)
                        {
                                RobotJointManager_SendMITUsingCache(right_leg_joint_order[i]);
                        }
                }
                else
                { //void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
                  RobotJointManager_SendMIT(ROBOT_JOINT_RIGHT_LEG_HIP_PITCH, 0.0f, 0.0f,0.0f, 0.0f,my_tor2);//right_pitch
                  RobotJointManager_SendMIT(ROBOT_JOINT_RIGHT_LEG_HIP_ROLL, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right_roll
                  RobotJointManager_SendMIT(ROBOT_JOINT_RIGHT_LEG_HIP_YAW, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right_yaw
                  RobotJointManager_SendMIT(ROBOT_JOINT_RIGHT_LEG_KNEE, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right_calf
                  RobotJointManager_SendMIT(ROBOT_JOINT_RIGHT_LEG_ANKLE_PITCH, my_pos2, my_vel2,my_kp2, my_kd2,0.0f);//right_foot
          RobotJointManager_SendMIT(ROBOT_JOINT_RIGHT_LEG_ANKLE_ROLL, my_pos2, my_vel2,my_kp2, my_kd2,0.0f);//right_foot
          RobotJointManager_SendMIT(ROBOT_JOINT_RIGHT_LEG_TOE, my_pos2, my_vel2,my_kp2, my_kd2,0.0f);//right_foot
                }
                if(a==1)
                {
                        RobotJointManager_SaveZero(ROBOT_JOINT_RIGHT_LEG_KNEE);
                        osDelay(CHASSR_TIME);
                }
                osDelay(CHASSR_TIME);
        }
}

void ChassisR_init(chassis_t *chassis)
{
        RobotJointManager_RegisterJoint(ROBOT_JOINT_RIGHT_LEG_HIP_PITCH, &chassis->joint_motor[7], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_RIGHT_LEG_HIP_ROLL, &chassis->joint_motor[8], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_RIGHT_LEG_HIP_YAW, &chassis->joint_motor[9], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_RIGHT_LEG_KNEE, &chassis->joint_motor[10], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_RIGHT_LEG_ANKLE_PITCH, &chassis->joint_motor[11], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_RIGHT_LEG_ANKLE_ROLL, &chassis->joint_motor[12], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_RIGHT_LEG_TOE, &chassis->joint_motor[13], MIT_MODE);

        RobotJointManager_EnableLimb(ROBOT_LIMB_RIGHT_LEG, 10, 100);
}


/*
void ChassisR_init(chassis_t *chassis)
{
    // 初始化所有电机的参数结构体
    joint_motor_init(&chassis->joint_motor[7], 1, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[8], 2, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[9], 3, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[10], 5, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[11], 7, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[12], 8, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[13], 9, MIT_MODE);
    
    // 短暂延时，确保系统稳定
    osDelay(100);

    // 循环为所有电机发送一次使能指令
    // 注意：这里的数组索引是从7到13
    for(int i = 7; i <= 13; i++)
    {
        enable_motor_mode(&hfdcan1, chassis->joint_motor[i].para.id, chassis->joint_motor[i].mode);
        osDelay(20); // 留出20ms的间隔，防止CAN总线拥堵，也给电机响应时间
    }
    
    // 在这里，最好能有一个机制来查询并确认所有电机都已进入MIT模式
    // (这需要你的驱动库支持读取电机状态的功能)
}
*/

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





