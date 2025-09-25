#ifndef __BODY_TASK_H
#define __BODY_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "chassisL_task.h"
#include "INS_task.h"

typedef struct
{
    Joint_Motor_t neck_motor[3];
    Joint_Motor_t left_arm_motor[4];
    Joint_Motor_t right_arm_motor[4];
    Joint_Motor_t loin_motor;
    uint8_t start_flag;//־
} body_t;

extern void body_init(body_t *body);
extern void Body_task(void);
extern void Body_SetJointTarget(RobotJointId joint, float position, float velocity, float kp, float kd, float torque);
extern void Body_SetJointRamp(RobotJointId joint, float ramp);

#endif
