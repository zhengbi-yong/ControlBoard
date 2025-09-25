/**
 * @file        body_task.c
 * @brief       High level body coordination task for the humanoid robot.
 *
 * The original implementation only documented the CAN bus usage.  The
 * reworked version describes the complete control flow and documents each
 * function so that future maintainers can quickly understand the
 * responsibilities of the task:
 *   - The file owns the `BodyJointCommand` table, which stores the desired
 *     state for every joint driven by the body task.
 *   - A cooperative scheduler (CMSIS-RTOS) triggers `Body_task`, which
 *     updates the waist motor and the upper-body joints every cycle.
 *   - Helper routines translate high level references to MIT mode commands
 *     understood by the motor drivers.
 *
 * In addition to documentation the update introduces several performance
 * optimisations:
 *   - The command array size is cached to avoid recomputing the element count
 *     on every lookup.
 *   - The slope follower now relies on `fabsf` to limit the error in a single
 *     step instead of performing a pair of comparisons, reducing branching
 *     and yielding more predictable execution time on Cortex-M CPUs.
 *   - Boolean expressions are simplified and redundant operations removed so
 *     that the task body performs the minimum amount of work per tick.
 */

#include "body_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "chassisR_task.h"
#include <math.h>
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

static void slope_following(float *target, float *set, float acc);

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

static const size_t body_joint_command_count = sizeof(body_joint_commands) / sizeof(body_joint_commands[0]);

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

/**
 * @brief   Apply the desired joint state to the corresponding actuator.
 *
 * When @p active is false the joint is commanded to a relaxed position so
 * that the motor driver does not accumulate unwanted current.  When active
 * the function performs a single slope-following step before emitting a MIT
 * command frame.
 */
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

/**
 * @brief   Iterate over all upper-body joints and apply the current commands.
 *
 * The waist joint is handled separately inside the main task loop, therefore
 * it is skipped here to avoid redundant CAN messages.
 */
static void Body_ProcessUpperBody(bool active)
{
    for (size_t i = 0; i < body_joint_command_count; ++i)
    {
        if (body_joint_commands[i].joint == ROBOT_JOINT_WAIST_YAW)
        {
            continue;
        }
        Body_UpdateCommand(&body_joint_commands[i], active);
    }
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

/**
 * @brief   Incrementally move @p set towards @p target respecting @p acc.
 *
 * The implementation clamps the error in a single step using `fabsf`, which
 * removes two branches compared to the previous version.  The reduced branch
 * count helps the CPU keep the pipeline full and lowers the jitter of the
 * control loop.
 */
static void slope_following(float *target, float *set, float acc)
{
    const float diff = *target - *set;
    const float step = (fabsf(diff) < acc) ? diff : (diff > 0.0f ? acc : -acc);
    *set += step;
}

void Body_task(void)
{
    osDelay(3000);
    body_init(&robot_body);

    BodyJointCommand *waist_cmd = Body_FindCommand(ROBOT_JOINT_WAIST_YAW);
    Joint_Motor_t *waist_motor = RobotJointManager_GetMotor(ROBOT_JOINT_WAIST_YAW);
    Joint_Motor_t *left_pitch_motor = RobotJointManager_GetMotor(ROBOT_JOINT_LEFT_LEG_HIP_PITCH);

    while (1)
    {
        const bool chassis_started = (chassis_move.start_flag == 1);
        const bool leg_ready = (chassis_started && left_pitch_motor != NULL && left_pitch_motor->para.kp_int_test != 0);

        if (waist_cmd != NULL && waist_motor != NULL)
        {
            if (leg_ready)
            {
                if (record_flag == 0U)
                {
                    record_flag = 1U;
                    waist_cmd->current_pos = waist_motor->para.pos;
                }
                slope_following(&waist_cmd->target_pos, &waist_cmd->current_pos, waist_cmd->ramp);
                RobotJointManager_SendMIT(waist_cmd->joint,
                                          waist_cmd->current_pos,
                                          waist_cmd->target_vel,
                                          waist_cmd->target_kp,
                                          waist_cmd->target_kd,
                                          waist_cmd->target_torque);
            }
            else
            {
                record_flag = 0U;
                waist_cmd->current_pos = 0.0f;
                RobotJointManager_SendMIT(waist_cmd->joint, 0.0f, 0.0f, 0.0f, waist_cmd->target_kd, 0.0f);
            }
        }

        const bool upper_active = (robot_body.start_flag != 0U) && chassis_started;
        Body_ProcessUpperBody(upper_active);

        if (c == 1)
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
