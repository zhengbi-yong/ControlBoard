#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__
#include <stdbool.h>

#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

//4310
#define P_MIN1 -12.5f
#define P_MAX1 12.5f
#define V_MIN1 -30.0f
#define V_MAX1 30.0f
#define KP_MIN1 0.0f
#define KP_MAX1 500.0f
#define KD_MIN1 0.0f
#define KD_MAX1 5.0f
#define T_MIN1 -10.0f
#define T_MAX1 10.0f

//4340
#define P_MIN2 -12.5f
#define P_MAX2 12.5f
#define V_MIN2 -10.0f
#define V_MAX2 10.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -28.0f
#define T_MAX2 28.0f

//6006
#define P_MIN3 -12.5f
#define P_MAX3 12.5f
#define V_MIN3 -45.0f
#define V_MAX3 45.0f
#define KP_MIN3 0.0f
#define KP_MAX3 500.0f
#define KD_MIN3 0.0f
#define KD_MAX3 5.0f
#define T_MIN3 -12.0f
#define T_MAX3 12.0f

//8006
#define P_MIN4 -12.5f
#define P_MAX4 12.5f
#define V_MIN4 -45.0f
#define V_MAX4 45.0f
#define KP_MIN4 0.0f
#define KP_MAX4 500.0f
#define KD_MIN4 0.0f
#define KD_MAX4 5.0f
#define T_MIN4 -20.0f
#define T_MAX4 20.0f

//3507
#define P_MIN5 -12.5f
#define P_MAX5 12.5f
#define V_MIN5 -50.0f
#define V_MAX5 50.0f
#define KP_MIN5 0.0f
#define KP_MAX5 500.0f
#define KD_MIN5 0.0f
#define KD_MAX5 5.0f
#define T_MIN5 -10.0f
#define T_MAX5 10.0f

//10010l
#define P_MIN6 -12.5f
#define P_MAX6 12.5f
#define V_MIN6 -25.0f
#define V_MAX6 25.0f
#define KP_MIN6 0.0f
#define KP_MAX6 500.0f
#define KD_MIN6 0.0f
#define KD_MAX6 5.0f
#define T_MIN6 -200.0f
#define T_MAX6 200.0f

//6248p
#define P_MIN7 -12.566f
#define P_MAX7 12.566f
#define V_MIN7 -20.0f
#define V_MAX7 20.0f
#define KP_MIN7 0.0f
#define KP_MAX7 500.0f
#define KD_MIN7 0.0f
#define KD_MAX7 5.0f
#define T_MIN7 -120.0f
#define T_MAX7 120.0f

typedef struct 
{
	uint16_t id;
        uint16_t state;
        // Raw integer feedback values reported by the motor driver.
        int p_int;
        int v_int;
        int t_int;
        int kp_int;
        int kd_int;
	float pos;
	float vel;
        float tor;
        float kp;
        float kd;
        float Tmos;
        float Tcoil;

        float kp_test;
        float kd_test;
        float tor_set;
        float pos_set;
        float vel_set;
        // Cached command words used when replaying MIT-mode set points.
        int kp_int_test;
        int kd_int_test;
        int p_int_test;
        int v_int_test;
        int t_int_test;
}motor_fbpara_t;


typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;
}Joint_Motor_t ;

typedef struct
{
        uint16_t mode;
        float wheel_T; // Estimated wheel torque in newtons.

        motor_fbpara_t para;
}Wheel_Motor_t ;

typedef enum
{
    ROBOT_MOTOR_DM4310 = 0,
    ROBOT_MOTOR_DM4340,
    ROBOT_MOTOR_DM6006,
    ROBOT_MOTOR_DM8006,
    ROBOT_MOTOR_DM3507,
    ROBOT_MOTOR_DM10010L,
    ROBOT_MOTOR_DM6248P,
    ROBOT_MOTOR_UNKNOWN
} RobotMotorModel;

typedef enum
{
    ROBOT_LIMB_WAIST = 0,
    ROBOT_LIMB_NECK,
    ROBOT_LIMB_LEFT_ARM,
    ROBOT_LIMB_RIGHT_ARM,
    ROBOT_LIMB_LEFT_LEG,
    ROBOT_LIMB_RIGHT_LEG,
    ROBOT_LIMB_COUNT
} RobotLimb;

typedef enum
{
    ROBOT_JOINT_WAIST_YAW = 0,
    ROBOT_JOINT_NECK_YAW,
    ROBOT_JOINT_NECK_PITCH,
    ROBOT_JOINT_NECK_ROLL,
    ROBOT_JOINT_LEFT_ARM_SHOULDER_PITCH,
    ROBOT_JOINT_LEFT_ARM_SHOULDER_ROLL,
    ROBOT_JOINT_LEFT_ARM_ELBOW,
    ROBOT_JOINT_LEFT_ARM_WRIST,
    ROBOT_JOINT_RIGHT_ARM_SHOULDER_PITCH,
    ROBOT_JOINT_RIGHT_ARM_SHOULDER_ROLL,
    ROBOT_JOINT_RIGHT_ARM_ELBOW,
    ROBOT_JOINT_RIGHT_ARM_WRIST,
    ROBOT_JOINT_LEFT_LEG_HIP_PITCH,
    ROBOT_JOINT_LEFT_LEG_HIP_YAW,
    ROBOT_JOINT_LEFT_LEG_HIP_ROLL,
    ROBOT_JOINT_LEFT_LEG_KNEE,
    ROBOT_JOINT_LEFT_LEG_ANKLE_PITCH,
    ROBOT_JOINT_LEFT_LEG_ANKLE_ROLL,
    ROBOT_JOINT_LEFT_LEG_TOE,
    ROBOT_JOINT_RIGHT_LEG_HIP_PITCH,
    ROBOT_JOINT_RIGHT_LEG_HIP_YAW,
    ROBOT_JOINT_RIGHT_LEG_HIP_ROLL,
    ROBOT_JOINT_RIGHT_LEG_KNEE,
    ROBOT_JOINT_RIGHT_LEG_ANKLE_PITCH,
    ROBOT_JOINT_RIGHT_LEG_ANKLE_ROLL,
    ROBOT_JOINT_RIGHT_LEG_TOE,
    ROBOT_JOINT_COUNT
} RobotJointId;

typedef struct
{
    RobotJointId joint_id;
    RobotMotorModel model;
    RobotLimb limb;
    uint16_t command_id;
    uint16_t feedback_id;
    uint16_t default_mode;
    hcan_t *bus;
    const char *name;
} RobotJointHardwareConfig;

typedef struct
{
    RobotJointId joint_id;
    Joint_Motor_t *joint;
    RobotMotorModel model;
    RobotLimb limb;
    hcan_t *bus;
    uint16_t command_id;
    uint16_t feedback_id;
    uint16_t mode;
    const char *name;
} RobotJointConfig;


extern void dm6006_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void dm4340_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void dm8006_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void dm3507_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void dm10010l_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void dm6248p_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);

extern void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

//4310
extern void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
extern void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);
extern void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float _vel);

//4340
extern void mit_ctrl2(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
//6006
extern void mit_ctrl3(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
//8006
extern void mit_ctrl4(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
//3507
extern void mit_ctrl5(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);


extern void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode);
extern void wheel_motor_init(Wheel_Motor_t *motor,uint16_t id,uint16_t mode);
	
extern float Hex_To_Float(uint32_t *Byte,int num);   /**< Reinterpret a 32-bit word as a float. */
extern uint32_t FloatTohex(float HEX);              /**< Reinterpret a float as a raw 32-bit word. */

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);


extern void save_motor_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);


extern void dm4310_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data);
extern void dm4340_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data);
extern void dm6006_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data);
extern void dm8006_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data);
extern void dm3507_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data);
extern void dm10010l_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data);
extern void dm6248p_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data);

extern void mit_ctrl_test(hcan_t* hcan, uint16_t motor_id,Joint_Motor_t *motor);

extern void dm4310_fbdata_init(Joint_Motor_t *motor);
extern void dm4340_fbdata_init(Joint_Motor_t *motor);
extern void dm6006_fbdata_init(Joint_Motor_t *motor);
extern void dm8006_fbdata_init(Joint_Motor_t *motor);
extern void dm3507_fbdata_init(Joint_Motor_t *motor);
extern void dm10010l_fbdata_init(Joint_Motor_t *motor);
extern void dm6248p_fbdata_init(Joint_Motor_t *motor);

extern const RobotJointHardwareConfig *RobotJointHardware_GetConfig(RobotJointId joint_id);
extern bool RobotJointHardware_SetConfig(const RobotJointHardwareConfig *config);

extern void RobotJointManager_Reset(void);
extern bool RobotJointManager_Register(const RobotJointConfig *config);
extern bool RobotJointManager_RegisterJoint(RobotJointId joint_id, Joint_Motor_t *joint, uint16_t override_mode);
extern Joint_Motor_t *RobotJointManager_GetMotor(RobotJointId joint_id);
extern bool RobotJointManager_SendMIT(RobotJointId joint_id, float pos, float vel, float kp, float kd, float torq);
extern bool RobotJointManager_SendMITUsingCache(RobotJointId joint_id);
extern bool RobotJointManager_SaveZero(RobotJointId joint_id);
extern void RobotJointManager_EnableLimb(RobotLimb limb, uint8_t repeat, uint16_t delay_ms);
extern void RobotJointManager_EnableAll(uint8_t repeat, uint16_t delay_ms);
extern void RobotJointManager_HandleFeedback(hcan_t *bus, uint16_t feedback_id, uint8_t *data, uint32_t len);
#endif /* __DM4310_DRV_H__ */



