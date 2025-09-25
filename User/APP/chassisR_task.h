#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "control_board_profile.h"


typedef struct
{
  Joint_Motor_t joint_motor[CONTROL_BOARD_MAX_LEG_JOINTS];

        uint8_t start_flag;//־

} chassis_t;


extern void ChassisR_init(chassis_t *chassis);
extern void ChassisR_task(void);

extern void mySaturate(float *in,float min,float max);


#endif
