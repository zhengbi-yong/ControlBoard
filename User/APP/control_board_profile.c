#include "control_board_profile.h"

/* Static configuration tables for each supported profile. */

static const ControlBoardBodyJointConfig kNeckBodyJoints[] = {
    {ROBOT_JOINT_NECK_YAW, 60.0f, 0.8f, 0.0f, 0.002f},
    {ROBOT_JOINT_NECK_PITCH, 60.0f, 0.8f, 0.0f, 0.002f},
    {ROBOT_JOINT_NECK_ROLL, 60.0f, 0.8f, 0.0f, 0.002f},
};

static const RobotLimb kNeckLimbs[] = {
    ROBOT_LIMB_NECK,
};

static const RobotJointId kNeckTelemetry[] = {
    ROBOT_JOINT_NECK_YAW,
    ROBOT_JOINT_NECK_PITCH,
    ROBOT_JOINT_NECK_ROLL,
};

static const ControlBoardBodyJointConfig kLeftArmBodyJoints[] = {
    {ROBOT_JOINT_LEFT_ARM_SHOULDER_PITCH, 40.0f, 0.6f, 0.0f, 0.003f},
    {ROBOT_JOINT_LEFT_ARM_SHOULDER_ROLL, 35.0f, 0.6f, 0.0f, 0.003f},
    {ROBOT_JOINT_LEFT_ARM_ELBOW, 35.0f, 0.5f, 0.0f, 0.003f},
    {ROBOT_JOINT_LEFT_ARM_WRIST, 30.0f, 0.5f, 0.0f, 0.003f},
};

static const RobotLimb kLeftArmLimbs[] = {
    ROBOT_LIMB_LEFT_ARM,
};

static const RobotJointId kLeftArmTelemetry[] = {
    ROBOT_JOINT_LEFT_ARM_SHOULDER_PITCH,
    ROBOT_JOINT_LEFT_ARM_SHOULDER_ROLL,
    ROBOT_JOINT_LEFT_ARM_ELBOW,
    ROBOT_JOINT_LEFT_ARM_WRIST,
};

static const ControlBoardBodyJointConfig kRightArmBodyJoints[] = {
    {ROBOT_JOINT_RIGHT_ARM_SHOULDER_PITCH, 40.0f, 0.6f, 0.0f, 0.003f},
    {ROBOT_JOINT_RIGHT_ARM_SHOULDER_ROLL, 35.0f, 0.6f, 0.0f, 0.003f},
    {ROBOT_JOINT_RIGHT_ARM_ELBOW, 35.0f, 0.5f, 0.0f, 0.003f},
    {ROBOT_JOINT_RIGHT_ARM_WRIST, 30.0f, 0.5f, 0.0f, 0.003f},
};

static const RobotLimb kRightArmLimbs[] = {
    ROBOT_LIMB_RIGHT_ARM,
};

static const RobotJointId kRightArmTelemetry[] = {
    ROBOT_JOINT_RIGHT_ARM_SHOULDER_PITCH,
    ROBOT_JOINT_RIGHT_ARM_SHOULDER_ROLL,
    ROBOT_JOINT_RIGHT_ARM_ELBOW,
    ROBOT_JOINT_RIGHT_ARM_WRIST,
};

static const ControlBoardBodyJointConfig kWaistBodyJoints[] = {
    {ROBOT_JOINT_WAIST_YAW, 90.0f, 1.1f, 0.0f, 0.001f},
};

static const RobotLimb kWaistLimbs[] = {
    ROBOT_LIMB_WAIST,
};

static const RobotJointId kWaistTelemetry[] = {
    ROBOT_JOINT_WAIST_YAW,
};

static const RobotJointId kLeftLegJointOrder[] = {
    ROBOT_JOINT_LEFT_LEG_HIP_PITCH,
    ROBOT_JOINT_LEFT_LEG_HIP_YAW,
    ROBOT_JOINT_LEFT_LEG_HIP_ROLL,
    ROBOT_JOINT_LEFT_LEG_KNEE,
    ROBOT_JOINT_LEFT_LEG_ANKLE_PITCH,
    ROBOT_JOINT_LEFT_LEG_ANKLE_ROLL,
};

static const RobotLimb kLeftLegLimbs[] = {
    ROBOT_LIMB_LEFT_LEG,
};

static const RobotJointId kLeftLegTelemetry[] = {
    ROBOT_JOINT_LEFT_LEG_HIP_PITCH,
    ROBOT_JOINT_LEFT_LEG_HIP_YAW,
    ROBOT_JOINT_LEFT_LEG_HIP_ROLL,
    ROBOT_JOINT_LEFT_LEG_KNEE,
    ROBOT_JOINT_LEFT_LEG_ANKLE_PITCH,
    ROBOT_JOINT_LEFT_LEG_ANKLE_ROLL,
};

static const RobotJointId kRightLegJointOrder[] = {
    ROBOT_JOINT_RIGHT_LEG_HIP_PITCH,
    ROBOT_JOINT_RIGHT_LEG_HIP_ROLL,
    ROBOT_JOINT_RIGHT_LEG_HIP_YAW,
    ROBOT_JOINT_RIGHT_LEG_KNEE,
    ROBOT_JOINT_RIGHT_LEG_ANKLE_PITCH,
    ROBOT_JOINT_RIGHT_LEG_ANKLE_ROLL,
};

static const RobotLimb kRightLegLimbs[] = {
    ROBOT_LIMB_RIGHT_LEG,
};

static const RobotJointId kRightLegTelemetry[] = {
    ROBOT_JOINT_RIGHT_LEG_HIP_PITCH,
    ROBOT_JOINT_RIGHT_LEG_HIP_ROLL,
    ROBOT_JOINT_RIGHT_LEG_HIP_YAW,
    ROBOT_JOINT_RIGHT_LEG_KNEE,
    ROBOT_JOINT_RIGHT_LEG_ANKLE_PITCH,
    ROBOT_JOINT_RIGHT_LEG_ANKLE_ROLL,
};

static const ControlBoardProfile kProfiles[CONTROL_BOARD_PROFILE_COUNT] = {
    [CONTROL_BOARD_PROFILE_NECK] = {
        .name = "Neck",
        .enable_body_task = true,
        .enable_left_leg_task = false,
        .enable_right_leg_task = false,
        .body_joints = kNeckBodyJoints,
        .body_joint_count = sizeof(kNeckBodyJoints) / sizeof(kNeckBodyJoints[0]),
        .body_limbs = kNeckLimbs,
        .body_limb_count = sizeof(kNeckLimbs) / sizeof(kNeckLimbs[0]),
        .telemetry_joints = kNeckTelemetry,
        .telemetry_joint_count = sizeof(kNeckTelemetry) / sizeof(kNeckTelemetry[0]),
    },
    [CONTROL_BOARD_PROFILE_LEFT_ARM] = {
        .name = "Left arm",
        .enable_body_task = true,
        .enable_left_leg_task = false,
        .enable_right_leg_task = false,
        .body_joints = kLeftArmBodyJoints,
        .body_joint_count = sizeof(kLeftArmBodyJoints) / sizeof(kLeftArmBodyJoints[0]),
        .body_limbs = kLeftArmLimbs,
        .body_limb_count = sizeof(kLeftArmLimbs) / sizeof(kLeftArmLimbs[0]),
        .telemetry_joints = kLeftArmTelemetry,
        .telemetry_joint_count = sizeof(kLeftArmTelemetry) / sizeof(kLeftArmTelemetry[0]),
    },
    [CONTROL_BOARD_PROFILE_RIGHT_ARM] = {
        .name = "Right arm",
        .enable_body_task = true,
        .enable_left_leg_task = false,
        .enable_right_leg_task = false,
        .body_joints = kRightArmBodyJoints,
        .body_joint_count = sizeof(kRightArmBodyJoints) / sizeof(kRightArmBodyJoints[0]),
        .body_limbs = kRightArmLimbs,
        .body_limb_count = sizeof(kRightArmLimbs) / sizeof(kRightArmLimbs[0]),
        .telemetry_joints = kRightArmTelemetry,
        .telemetry_joint_count = sizeof(kRightArmTelemetry) / sizeof(kRightArmTelemetry[0]),
    },
    [CONTROL_BOARD_PROFILE_LEFT_LEG] = {
        .name = "Left leg",
        .enable_body_task = false,
        .enable_left_leg_task = true,
        .enable_right_leg_task = false,
        .left_leg_joint_order = kLeftLegJointOrder,
        .left_leg_joint_count = sizeof(kLeftLegJointOrder) / sizeof(kLeftLegJointOrder[0]),
        .left_leg_limbs = kLeftLegLimbs,
        .left_leg_limb_count = sizeof(kLeftLegLimbs) / sizeof(kLeftLegLimbs[0]),
        .telemetry_joints = kLeftLegTelemetry,
        .telemetry_joint_count = sizeof(kLeftLegTelemetry) / sizeof(kLeftLegTelemetry[0]),
    },
    [CONTROL_BOARD_PROFILE_RIGHT_LEG] = {
        .name = "Right leg",
        .enable_body_task = false,
        .enable_left_leg_task = false,
        .enable_right_leg_task = true,
        .right_leg_joint_order = kRightLegJointOrder,
        .right_leg_joint_count = sizeof(kRightLegJointOrder) / sizeof(kRightLegJointOrder[0]),
        .right_leg_limbs = kRightLegLimbs,
        .right_leg_limb_count = sizeof(kRightLegLimbs) / sizeof(kRightLegLimbs[0]),
        .telemetry_joints = kRightLegTelemetry,
        .telemetry_joint_count = sizeof(kRightLegTelemetry) / sizeof(kRightLegTelemetry[0]),
    },
    [CONTROL_BOARD_PROFILE_WAIST] = {
        .name = "Waist",
        .enable_body_task = true,
        .enable_left_leg_task = false,
        .enable_right_leg_task = false,
        .body_joints = kWaistBodyJoints,
        .body_joint_count = sizeof(kWaistBodyJoints) / sizeof(kWaistBodyJoints[0]),
        .body_limbs = kWaistLimbs,
        .body_limb_count = sizeof(kWaistLimbs) / sizeof(kWaistLimbs[0]),
        .telemetry_joints = kWaistTelemetry,
        .telemetry_joint_count = sizeof(kWaistTelemetry) / sizeof(kWaistTelemetry[0]),
    },
};

const ControlBoardProfile *ControlBoardProfile_GetActive(void)
{
    return &kProfiles[CONTROL_BOARD_PROFILE];
}

const char *ControlBoardProfile_GetName(void)
{
    return ControlBoardProfile_GetActive()->name;
}
