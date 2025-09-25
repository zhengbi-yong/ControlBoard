/**
  *********************************************************************
  * @file      body_task.c/h
  * @brief     该任务控制腰部的一个电机，是DM6006，这个电机挂载在can3总线上
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "body_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "chassisR_task.h"
#include <stdbool.h>
#include <stddef.h>

extern INS_t INS;
extern chassis_t chassis_move;
body_t robot_body;

uint32_t BODY_TIME=1;
int c=0;
uint8_t record_flag=0;

typedef struct
{
    RobotJointId joint;
    float target_pos;
    float target_vel;
    float target_kp;
    float target_kd;
    float target_torque;
    float ramp;
    float current_pos;
} BodyJointCommand;

static BodyJointCommand body_joint_commands[] = {
    {ROBOT_JOINT_WAIST_YAW, 0.0f, 0.0f, 90.0f, 1.1f, 0.0f, 0.001f, 0.0f},
    {ROBOT_JOINT_NECK_YAW, 0.0f, 0.0f, 60.0f, 0.8f, 0.0f, 0.002f, 0.0f},
    {ROBOT_JOINT_NECK_PITCH, 0.0f, 0.0f, 60.0f, 0.8f, 0.0f, 0.002f, 0.0f},
    {ROBOT_JOINT_NECK_ROLL, 0.0f, 0.0f, 60.0f, 0.8f, 0.0f, 0.002f, 0.0f},
    {ROBOT_JOINT_LEFT_ARM_SHOULDER_PITCH, 0.0f, 0.0f, 40.0f, 0.6f, 0.0f, 0.003f, 0.0f},
    {ROBOT_JOINT_LEFT_ARM_SHOULDER_ROLL, 0.0f, 0.0f, 35.0f, 0.6f, 0.0f, 0.003f, 0.0f},
    {ROBOT_JOINT_LEFT_ARM_ELBOW, 0.0f, 0.0f, 35.0f, 0.5f, 0.0f, 0.003f, 0.0f},
    {ROBOT_JOINT_LEFT_ARM_WRIST, 0.0f, 0.0f, 30.0f, 0.5f, 0.0f, 0.003f, 0.0f},
    {ROBOT_JOINT_RIGHT_ARM_SHOULDER_PITCH, 0.0f, 0.0f, 40.0f, 0.6f, 0.0f, 0.003f, 0.0f},
    {ROBOT_JOINT_RIGHT_ARM_SHOULDER_ROLL, 0.0f, 0.0f, 35.0f, 0.6f, 0.0f, 0.003f, 0.0f},
    {ROBOT_JOINT_RIGHT_ARM_ELBOW, 0.0f, 0.0f, 35.0f, 0.5f, 0.0f, 0.003f, 0.0f},
    {ROBOT_JOINT_RIGHT_ARM_WRIST, 0.0f, 0.0f, 30.0f, 0.5f, 0.0f, 0.003f, 0.0f},
};

static BodyJointCommand *Body_FindCommand(RobotJointId joint)
{
    for(size_t i = 0; i < sizeof(body_joint_commands)/sizeof(body_joint_commands[0]); ++i)
    {
        if(body_joint_commands[i].joint == joint)
        {
            return &body_joint_commands[i];
        }
    }
    return NULL;
}

static void Body_UpdateCommand(BodyJointCommand *command, bool active)
{
    if(command == NULL)
    {
        return;
    }

    if(!active)
    {
        command->current_pos = 0.0f;
        RobotJointManager_SendMIT(command->joint, 0.0f, 0.0f, 0.0f, command->target_kd, 0.0f);
        return;
    }

    slope_following(&command->target_pos, &command->current_pos, command->ramp);
    RobotJointManager_SendMIT(command->joint, command->current_pos, command->target_vel, command->target_kp, command->target_kd, command->target_torque);
}

static void Body_ProcessUpperBody(bool active)
{
    for(size_t i = 0; i < sizeof(body_joint_commands)/sizeof(body_joint_commands[0]); ++i)
    {
        if(body_joint_commands[i].joint == ROBOT_JOINT_WAIST_YAW)
        {
            continue;
        }
        Body_UpdateCommand(&body_joint_commands[i], active);
    }
}

void Body_SetJointTarget(RobotJointId joint, float position, float velocity, float kp, float kd, float torque)
{
    BodyJointCommand *cmd = Body_FindCommand(joint);
    if(cmd == NULL)
    {
        return;
    }
    cmd->target_pos = position;
    cmd->target_vel = velocity;
    cmd->target_kp = kp;
    cmd->target_kd = kd;
    cmd->target_torque = torque;
}

void Body_SetJointRamp(RobotJointId joint, float ramp)
{
    BodyJointCommand *cmd = Body_FindCommand(joint);
    if(cmd == NULL)
    {
        return;
    }
    cmd->ramp = ramp;
}

void slope_following(float *target,float *set,float acc)
{
        if(*target > *set)
        {
                *set = *set + acc;
                if(*set >= *target)
                {
                        *set = *target;
                }
        }
        else if(*target < *set)
        {
                *set = *set - acc;
                if(*set <= *target)
                {
                        *set = *target;
                }
        }
}

void Body_task(void)
{
        osDelay(3000);
  body_init(&robot_body);

        BodyJointCommand *waist_cmd = Body_FindCommand(ROBOT_JOINT_WAIST_YAW);
        Joint_Motor_t *waist_motor = RobotJointManager_GetMotor(ROBOT_JOINT_WAIST_YAW);
        Joint_Motor_t *left_pitch_motor = RobotJointManager_GetMotor(ROBOT_JOINT_LEFT_LEG_HIP_PITCH);

        while(1)
        {
                bool leg_ready = false;
                if(left_pitch_motor != NULL && chassis_move.start_flag==1)
                {
                        leg_ready = (left_pitch_motor->para.kp_int_test != 0);
                }

                if(waist_cmd != NULL && waist_motor != NULL)
                {
                        if(leg_ready)
                        {
                                if(record_flag==0)
                                {
                                        record_flag=1;
                                        waist_cmd->current_pos = waist_motor->para.pos;
                                }
                                slope_following(&waist_cmd->target_pos, &waist_cmd->current_pos, waist_cmd->ramp);
                                RobotJointManager_SendMIT(waist_cmd->joint, waist_cmd->current_pos, waist_cmd->target_vel, waist_cmd->target_kp, waist_cmd->target_kd, waist_cmd->target_torque);
                        }
                        else
                        {
                                record_flag=0;
                                waist_cmd->current_pos = 0.0f;
                                RobotJointManager_SendMIT(waist_cmd->joint, 0.0f, 0.0f, 0.0f, waist_cmd->target_kd, 0.0f);
                        }
                }

                bool upper_active = (robot_body.start_flag != 0U) && (chassis_move.start_flag==1);
                Body_ProcessUpperBody(upper_active);

                if(c==1)
                {
                  RobotJointManager_SaveZero(ROBOT_JOINT_WAIST_YAW);
                  osDelay(BODY_TIME);
                }
                osDelay(BODY_TIME);
        }
}

void body_init(body_t *body)
{
        RobotJointManager_RegisterJoint(ROBOT_JOINT_WAIST_YAW, &body->loin_motor, MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_NECK_YAW, &body->neck_motor[0], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_NECK_PITCH, &body->neck_motor[1], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_NECK_ROLL, &body->neck_motor[2], MIT_MODE);

        RobotJointManager_RegisterJoint(ROBOT_JOINT_LEFT_ARM_SHOULDER_PITCH, &body->left_arm_motor[0], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_LEFT_ARM_SHOULDER_ROLL, &body->left_arm_motor[1], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_LEFT_ARM_ELBOW, &body->left_arm_motor[2], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_LEFT_ARM_WRIST, &body->left_arm_motor[3], MIT_MODE);

        RobotJointManager_RegisterJoint(ROBOT_JOINT_RIGHT_ARM_SHOULDER_PITCH, &body->right_arm_motor[0], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_RIGHT_ARM_SHOULDER_ROLL, &body->right_arm_motor[1], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_RIGHT_ARM_ELBOW, &body->right_arm_motor[2], MIT_MODE);
        RobotJointManager_RegisterJoint(ROBOT_JOINT_RIGHT_ARM_WRIST, &body->right_arm_motor[3], MIT_MODE);

        RobotJointManager_EnableLimb(ROBOT_LIMB_WAIST, 10, 20);
        RobotJointManager_EnableLimb(ROBOT_LIMB_NECK, 10, 20);
        RobotJointManager_EnableLimb(ROBOT_LIMB_LEFT_ARM, 10, 20);
        RobotJointManager_EnableLimb(ROBOT_LIMB_RIGHT_ARM, 10, 20);

        body->start_flag = 1;
}
