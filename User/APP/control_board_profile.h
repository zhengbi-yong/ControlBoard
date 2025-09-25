#ifndef USER_APP_CONTROL_BOARD_PROFILE_H
#define USER_APP_CONTROL_BOARD_PROFILE_H

#include <stdbool.h>
#include <stddef.h>

#include "dm4310_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Compile-time identifiers for the supported control-board layouts.
 */
typedef enum
{
    CONTROL_BOARD_PROFILE_NECK = 0,
    CONTROL_BOARD_PROFILE_LEFT_ARM,
    CONTROL_BOARD_PROFILE_RIGHT_ARM,
    CONTROL_BOARD_PROFILE_LEFT_LEG,
    CONTROL_BOARD_PROFILE_RIGHT_LEG,
    CONTROL_BOARD_PROFILE_WAIST,
    CONTROL_BOARD_PROFILE_COUNT
} ControlBoardProfileId;

#ifndef CONTROL_BOARD_PROFILE
/**
 * @brief Selected control-board profile.
 *
 * Edit this macro to switch the firmware between the available profiles.  The
 * remainder of the code automatically adapts to the chosen layout without the
 * need to touch any other source file.
 */
#define CONTROL_BOARD_PROFILE CONTROL_BOARD_PROFILE_NECK
#endif

/** Maximum number of joints handled by the body task. */
#define CONTROL_BOARD_MAX_BODY_JOINTS 12U
/** Maximum number of joints handled by a single leg controller. */
#define CONTROL_BOARD_MAX_LEG_JOINTS 7U
/** Maximum number of joints streamed to the host interfaces. */
#define CONTROL_BOARD_MAX_TELEMETRY_JOINTS 14U

/**
 * @brief Default MIT parameters associated with a body joint.
 */
typedef struct
{
    RobotJointId joint;   /**< Logical joint identifier. */
    float kp;             /**< Default proportional gain. */
    float kd;             /**< Default derivative gain. */
    float torque;         /**< Static torque bias. */
    float ramp;           /**< Position ramp limit per control tick. */
} ControlBoardBodyJointConfig;

/**
 * @brief Aggregated description of the control-board behaviour.
 */
typedef struct
{
    const char *name;                                         /**< Human readable profile name. */
    bool enable_body_task;                                    /**< True when the upper-body task is required. */
    bool enable_left_leg_task;                                /**< True when the left-leg controller should run. */
    bool enable_right_leg_task;                               /**< True when the right-leg controller should run. */

    const ControlBoardBodyJointConfig *body_joints;           /**< Body joints driven by this firmware image. */
    size_t body_joint_count;                                  /**< Number of valid body joint entries. */
    const RobotLimb *body_limbs;                              /**< Limbs enabled together with the body joints. */
    size_t body_limb_count;                                   /**< Number of limbs in @ref body_limbs. */

    const RobotJointId *left_leg_joint_order;                 /**< Order used when commanding the left leg. */
    size_t left_leg_joint_count;                              /**< Number of joints driven in the left leg. */
    const RobotLimb *left_leg_limbs;                          /**< Limbs enabled for the left-leg controller. */
    size_t left_leg_limb_count;                               /**< Number of elements in @ref left_leg_limbs. */

    const RobotJointId *right_leg_joint_order;                /**< Order used when commanding the right leg. */
    size_t right_leg_joint_count;                             /**< Number of joints driven in the right leg. */
    const RobotLimb *right_leg_limbs;                         /**< Limbs enabled for the right-leg controller. */
    size_t right_leg_limb_count;                              /**< Number of elements in @ref right_leg_limbs. */

    const RobotJointId *telemetry_joints;                     /**< Joints exported over UART/USB. */
    size_t telemetry_joint_count;                             /**< Number of elements in @ref telemetry_joints. */
} ControlBoardProfile;

/**
 * @brief Obtain the configuration of the active control-board profile.
 */
const ControlBoardProfile *ControlBoardProfile_GetActive(void);

/**
 * @brief Utility returning the human readable profile name.
 */
const char *ControlBoardProfile_GetName(void);

/* Convenience macros exposing compile-time booleans for conditional code. */
#if CONTROL_BOARD_PROFILE == CONTROL_BOARD_PROFILE_NECK
#define CONTROL_BOARD_PROFILE_ENABLE_BODY_TASK 1
#define CONTROL_BOARD_PROFILE_ENABLE_LEFT_LEG_TASK 0
#define CONTROL_BOARD_PROFILE_ENABLE_RIGHT_LEG_TASK 0
#elif CONTROL_BOARD_PROFILE == CONTROL_BOARD_PROFILE_LEFT_ARM
#define CONTROL_BOARD_PROFILE_ENABLE_BODY_TASK 1
#define CONTROL_BOARD_PROFILE_ENABLE_LEFT_LEG_TASK 0
#define CONTROL_BOARD_PROFILE_ENABLE_RIGHT_LEG_TASK 0
#elif CONTROL_BOARD_PROFILE == CONTROL_BOARD_PROFILE_RIGHT_ARM
#define CONTROL_BOARD_PROFILE_ENABLE_BODY_TASK 1
#define CONTROL_BOARD_PROFILE_ENABLE_LEFT_LEG_TASK 0
#define CONTROL_BOARD_PROFILE_ENABLE_RIGHT_LEG_TASK 0
#elif CONTROL_BOARD_PROFILE == CONTROL_BOARD_PROFILE_LEFT_LEG
#define CONTROL_BOARD_PROFILE_ENABLE_BODY_TASK 0
#define CONTROL_BOARD_PROFILE_ENABLE_LEFT_LEG_TASK 1
#define CONTROL_BOARD_PROFILE_ENABLE_RIGHT_LEG_TASK 0
#elif CONTROL_BOARD_PROFILE == CONTROL_BOARD_PROFILE_RIGHT_LEG
#define CONTROL_BOARD_PROFILE_ENABLE_BODY_TASK 0
#define CONTROL_BOARD_PROFILE_ENABLE_LEFT_LEG_TASK 0
#define CONTROL_BOARD_PROFILE_ENABLE_RIGHT_LEG_TASK 1
#elif CONTROL_BOARD_PROFILE == CONTROL_BOARD_PROFILE_WAIST
#define CONTROL_BOARD_PROFILE_ENABLE_BODY_TASK 1
#define CONTROL_BOARD_PROFILE_ENABLE_LEFT_LEG_TASK 0
#define CONTROL_BOARD_PROFILE_ENABLE_RIGHT_LEG_TASK 0
#else
#error "Unknown CONTROL_BOARD_PROFILE selection"
#endif

#ifdef __cplusplus
}
#endif

#endif /* USER_APP_CONTROL_BOARD_PROFILE_H */
