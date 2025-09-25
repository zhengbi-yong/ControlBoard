#ifndef __BODY_TASK_H
#define __BODY_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "chassisL_task.h"
#include "INS_task.h"

/**
 * @brief Container for all actuators driven by the body task.
 *
 * The arrays mirror the mechanical structure of the robot and allow the task
 * to address the joints by semantic identifiers.  `start_flag` becomes true
 * once the upper-body is authorised to move.
 */
typedef struct
{
    Joint_Motor_t neck_motor[3];
    Joint_Motor_t left_arm_motor[4];
    Joint_Motor_t right_arm_motor[4];
    Joint_Motor_t loin_motor;
    uint8_t start_flag;
} body_t;

void body_init(body_t *body);
void Body_task(void);
void Body_SetJointTarget(RobotJointId joint, float position, float velocity, float kp, float kd, float torque);
void Body_SetJointRamp(RobotJointId joint, float ramp);

#endif
