/**
 * @file        body_task.c
 * @brief       High level body coordination task for the humanoid robot.
 *
 * The task now relies on the control-board profile abstraction so that the same
 * firmware can be reused on the neck, waist or arm controllers without touching
 * any other source file.  All joints managed by the body task are described in
 * @ref control_board_profile.c together with their default MIT gains.
 */

#include "body_task.h"
#include "control_board_profile.h"

#include "cmsis_os.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

body_t robot_body;

uint32_t BODY_TIME = 1;

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

static BodyJointCommand body_joint_commands[CONTROL_BOARD_MAX_BODY_JOINTS];
static size_t body_joint_command_count = 0U;

static void slope_following(float *target, float *set, float acc);

static void Body_LoadProfileCommands(const ControlBoardProfile *profile)
{
    body_joint_command_count = profile->body_joint_count;
    for (size_t i = 0; i < body_joint_command_count; ++i)
    {
        const ControlBoardBodyJointConfig *cfg = &profile->body_joints[i];
        body_joint_commands[i].joint = cfg->joint;
        body_joint_commands[i].target_pos = 0.0f;
        body_joint_commands[i].target_vel = 0.0f;
        body_joint_commands[i].target_kp = cfg->kp;
        body_joint_commands[i].target_kd = cfg->kd;
        body_joint_commands[i].target_torque = cfg->torque;
        body_joint_commands[i].ramp = cfg->ramp;
        body_joint_commands[i].current_pos = 0.0f;
    }
}

static BodyJointCommand *Body_FindCommand(RobotJointId joint)
{
    for (size_t i = 0; i < body_joint_command_count; ++i)
    {
        if (body_joint_commands[i].joint == joint)
        {
            return &body_joint_commands[i];
        }
    }
    return NULL;
}

static void Body_EnableLimbs(const RobotLimb *limbs, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        RobotJointManager_EnableLimb(limbs[i], 10, 20);
    }
}

static void Body_UpdateCommand(BodyJointCommand *command, bool active)
{
    if (command == NULL)
    {
        return;
    }

    if (!active)
    {
        command->current_pos = 0.0f;
        RobotJointManager_SendMIT(command->joint, 0.0f, 0.0f, 0.0f, command->target_kd, 0.0f);
        return;
    }

    slope_following(&command->target_pos, &command->current_pos, command->ramp);
    RobotJointManager_SendMIT(command->joint,
                              command->current_pos,
                              command->target_vel,
                              command->target_kp,
                              command->target_kd,
                              command->target_torque);
}

void Body_SetJointTarget(RobotJointId joint, float position, float velocity, float kp, float kd, float torque)
{
    BodyJointCommand *cmd = Body_FindCommand(joint);
    if (cmd == NULL)
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
    if (cmd == NULL)
    {
        return;
    }
    cmd->ramp = ramp;
}

static void slope_following(float *target, float *set, float acc)
{
    const float diff = *target - *set;
    const float step = (fabsf(diff) < acc) ? diff : (diff > 0.0f ? acc : -acc);
    *set += step;
}

void Body_task(void)
{
    const ControlBoardProfile *profile = ControlBoardProfile_GetActive();
    osDelay(3000);
    body_init(&robot_body);

    while (1)
    {
        if (!profile->enable_body_task || body_joint_command_count == 0U)
        {
            osDelay(BODY_TIME);
            continue;
        }

        const bool body_active = (robot_body.start_flag != 0U);
        for (size_t i = 0; i < body_joint_command_count; ++i)
        {
            Body_UpdateCommand(&body_joint_commands[i], body_active);
        }

        osDelay(BODY_TIME);
    }
}

void body_init(body_t *body)
{
    const ControlBoardProfile *profile = ControlBoardProfile_GetActive();

    if (!profile->enable_body_task || profile->body_joint_count == 0U)
    {
        body_joint_command_count = 0U;
        body->start_flag = 0U;
        return;
    }

    Body_LoadProfileCommands(profile);

    for (size_t i = 0; i < body_joint_command_count; ++i)
    {
        RobotJointManager_RegisterJoint(body_joint_commands[i].joint, &body->motors[i], MIT_MODE);
    }

    Body_EnableLimbs(profile->body_limbs, profile->body_limb_count);

    body->start_flag = 1U;
}
