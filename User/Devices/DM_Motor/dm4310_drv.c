#include "dm4310_drv.h"

#include <string.h>

#include "fdcan.h"
#include "arm_math.h"
#include "cmsis_os.h"

#define ROBOT_JOINT_FEEDBACK_OFFSET (0x10U)

typedef struct
{
    bool registered;
    RobotJointConfig config;
} RobotJointEntry;

#define ROBOT_JOINT_MAX_BUS_COUNT (4U)
#define ROBOT_JOINT_FEEDBACK_LOOKUP_SIZE (0x200U)

typedef struct
{
    hcan_t *bus;
    RobotJointEntry *feedback_lookup[ROBOT_JOINT_FEEDBACK_LOOKUP_SIZE];
} RobotJointBusCache;

static RobotJointHardwareConfig g_joint_hw_table[ROBOT_JOINT_COUNT] = {
    [ROBOT_JOINT_WAIST_YAW] = {
        .joint_id = ROBOT_JOINT_WAIST_YAW,
        .model = ROBOT_MOTOR_DM6006,
        .limb = ROBOT_LIMB_WAIST,
        .command_id = 0x09,
        .feedback_id = 0x19,
        .default_mode = MIT_MODE,
        .bus = &hfdcan3,
        .name = "waist_yaw",
    },
    [ROBOT_JOINT_NECK_YAW] = {
        .joint_id = ROBOT_JOINT_NECK_YAW,
        .model = ROBOT_MOTOR_DM4310,
        .limb = ROBOT_LIMB_NECK,
        .command_id = 0x0A,
        .feedback_id = 0x1A,
        .default_mode = MIT_MODE,
        .bus = &hfdcan3,
        .name = "neck_yaw",
    },
    [ROBOT_JOINT_NECK_PITCH] = {
        .joint_id = ROBOT_JOINT_NECK_PITCH,
        .model = ROBOT_MOTOR_DM4310,
        .limb = ROBOT_LIMB_NECK,
        .command_id = 0x0B,
        .feedback_id = 0x1B,
        .default_mode = MIT_MODE,
        .bus = &hfdcan3,
        .name = "neck_pitch",
    },
    [ROBOT_JOINT_NECK_ROLL] = {
        .joint_id = ROBOT_JOINT_NECK_ROLL,
        .model = ROBOT_MOTOR_DM4310,
        .limb = ROBOT_LIMB_NECK,
        .command_id = 0x0C,
        .feedback_id = 0x1C,
        .default_mode = MIT_MODE,
        .bus = &hfdcan3,
        .name = "neck_roll",
    },
    [ROBOT_JOINT_LEFT_ARM_SHOULDER_PITCH] = {
        .joint_id = ROBOT_JOINT_LEFT_ARM_SHOULDER_PITCH,
        .model = ROBOT_MOTOR_DM4310,
        .limb = ROBOT_LIMB_LEFT_ARM,
        .command_id = 0x41,
        .feedback_id = 0x51,
        .default_mode = MIT_MODE,
        .bus = &hfdcan2,
        .name = "left_shoulder_pitch",
    },
    [ROBOT_JOINT_LEFT_ARM_SHOULDER_ROLL] = {
        .joint_id = ROBOT_JOINT_LEFT_ARM_SHOULDER_ROLL,
        .model = ROBOT_MOTOR_DM4310,
        .limb = ROBOT_LIMB_LEFT_ARM,
        .command_id = 0x42,
        .feedback_id = 0x52,
        .default_mode = MIT_MODE,
        .bus = &hfdcan2,
        .name = "left_shoulder_roll",
    },
    [ROBOT_JOINT_LEFT_ARM_ELBOW] = {
        .joint_id = ROBOT_JOINT_LEFT_ARM_ELBOW,
        .model = ROBOT_MOTOR_DM4310,
        .limb = ROBOT_LIMB_LEFT_ARM,
        .command_id = 0x43,
        .feedback_id = 0x53,
        .default_mode = MIT_MODE,
        .bus = &hfdcan2,
        .name = "left_elbow",
    },
    [ROBOT_JOINT_LEFT_ARM_WRIST] = {
        .joint_id = ROBOT_JOINT_LEFT_ARM_WRIST,
        .model = ROBOT_MOTOR_DM4310,
        .limb = ROBOT_LIMB_LEFT_ARM,
        .command_id = 0x44,
        .feedback_id = 0x54,
        .default_mode = MIT_MODE,
        .bus = &hfdcan2,
        .name = "left_wrist",
    },
    [ROBOT_JOINT_RIGHT_ARM_SHOULDER_PITCH] = {
        .joint_id = ROBOT_JOINT_RIGHT_ARM_SHOULDER_PITCH,
        .model = ROBOT_MOTOR_DM4310,
        .limb = ROBOT_LIMB_RIGHT_ARM,
        .command_id = 0x21,
        .feedback_id = 0x31,
        .default_mode = MIT_MODE,
        .bus = &hfdcan1,
        .name = "right_shoulder_pitch",
    },
    [ROBOT_JOINT_RIGHT_ARM_SHOULDER_ROLL] = {
        .joint_id = ROBOT_JOINT_RIGHT_ARM_SHOULDER_ROLL,
        .model = ROBOT_MOTOR_DM4310,
        .limb = ROBOT_LIMB_RIGHT_ARM,
        .command_id = 0x22,
        .feedback_id = 0x32,
        .default_mode = MIT_MODE,
        .bus = &hfdcan1,
        .name = "right_shoulder_roll",
    },
    [ROBOT_JOINT_RIGHT_ARM_ELBOW] = {
        .joint_id = ROBOT_JOINT_RIGHT_ARM_ELBOW,
        .model = ROBOT_MOTOR_DM4310,
        .limb = ROBOT_LIMB_RIGHT_ARM,
        .command_id = 0x23,
        .feedback_id = 0x33,
        .default_mode = MIT_MODE,
        .bus = &hfdcan1,
        .name = "right_elbow",
    },
    [ROBOT_JOINT_RIGHT_ARM_WRIST] = {
        .joint_id = ROBOT_JOINT_RIGHT_ARM_WRIST,
        .model = ROBOT_MOTOR_DM4310,
        .limb = ROBOT_LIMB_RIGHT_ARM,
        .command_id = 0x24,
        .feedback_id = 0x34,
        .default_mode = MIT_MODE,
        .bus = &hfdcan1,
        .name = "right_wrist",
    },
    [ROBOT_JOINT_LEFT_LEG_HIP_PITCH] = {
        .joint_id = ROBOT_JOINT_LEFT_LEG_HIP_PITCH,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_LEFT_LEG,
        .command_id = 0x01,
        .feedback_id = 0x15,
        .default_mode = MIT_MODE,
        .bus = &hfdcan2,
        .name = "left_hip_pitch",
    },
    [ROBOT_JOINT_LEFT_LEG_HIP_YAW] = {
        .joint_id = ROBOT_JOINT_LEFT_LEG_HIP_YAW,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_LEFT_LEG,
        .command_id = 0x02,
        .feedback_id = 0x14,
        .default_mode = MIT_MODE,
        .bus = &hfdcan2,
        .name = "left_hip_yaw",
    },
    [ROBOT_JOINT_LEFT_LEG_HIP_ROLL] = {
        .joint_id = ROBOT_JOINT_LEFT_LEG_HIP_ROLL,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_LEFT_LEG,
        .command_id = 0x03,
        .feedback_id = 0x13,
        .default_mode = MIT_MODE,
        .bus = &hfdcan2,
        .name = "left_hip_roll",
    },
    [ROBOT_JOINT_LEFT_LEG_KNEE] = {
        .joint_id = ROBOT_JOINT_LEFT_LEG_KNEE,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_LEFT_LEG,
        .command_id = 0x04,
        .feedback_id = 0x12,
        .default_mode = MIT_MODE,
        .bus = &hfdcan2,
        .name = "left_knee",
    },
    [ROBOT_JOINT_LEFT_LEG_ANKLE_PITCH] = {
        .joint_id = ROBOT_JOINT_LEFT_LEG_ANKLE_PITCH,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_LEFT_LEG,
        .command_id = 0x05,
        .feedback_id = 0x11,
        .default_mode = MIT_MODE,
        .bus = &hfdcan2,
        .name = "left_ankle_pitch",
    },
    [ROBOT_JOINT_LEFT_LEG_ANKLE_ROLL] = {
        .joint_id = ROBOT_JOINT_LEFT_LEG_ANKLE_ROLL,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_LEFT_LEG,
        .command_id = 0x06,
        .feedback_id = 0x16,
        .default_mode = MIT_MODE,
        .bus = &hfdcan2,
        .name = "left_ankle_roll",
    },
    [ROBOT_JOINT_LEFT_LEG_TOE] = {
        .joint_id = ROBOT_JOINT_LEFT_LEG_TOE,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_LEFT_LEG,
        .command_id = 0x07,
        .feedback_id = 0x17,
        .default_mode = MIT_MODE,
        .bus = &hfdcan2,
        .name = "left_toe",
    },
    [ROBOT_JOINT_RIGHT_LEG_HIP_PITCH] = {
        .joint_id = ROBOT_JOINT_RIGHT_LEG_HIP_PITCH,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_RIGHT_LEG,
        .command_id = 0x01,
        .feedback_id = 0x11,
        .default_mode = MIT_MODE,
        .bus = &hfdcan1,
        .name = "right_hip_pitch",
    },
    [ROBOT_JOINT_RIGHT_LEG_HIP_YAW] = {
        .joint_id = ROBOT_JOINT_RIGHT_LEG_HIP_YAW,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_RIGHT_LEG,
        .command_id = 0x0A,
        .feedback_id = 0x1A,
        .default_mode = MIT_MODE,
        .bus = &hfdcan1,
        .name = "right_hip_yaw",
    },
    [ROBOT_JOINT_RIGHT_LEG_HIP_ROLL] = {
        .joint_id = ROBOT_JOINT_RIGHT_LEG_HIP_ROLL,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_RIGHT_LEG,
        .command_id = 0x0B,
        .feedback_id = 0x12,
        .default_mode = MIT_MODE,
        .bus = &hfdcan1,
        .name = "right_hip_roll",
    },
    [ROBOT_JOINT_RIGHT_LEG_KNEE] = {
        .joint_id = ROBOT_JOINT_RIGHT_LEG_KNEE,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_RIGHT_LEG,
        .command_id = 0x05,
        .feedback_id = 0x15,
        .default_mode = MIT_MODE,
        .bus = &hfdcan1,
        .name = "right_knee",
    },
    [ROBOT_JOINT_RIGHT_LEG_ANKLE_PITCH] = {
        .joint_id = ROBOT_JOINT_RIGHT_LEG_ANKLE_PITCH,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_RIGHT_LEG,
        .command_id = 0x07,
        .feedback_id = 0x17,
        .default_mode = MIT_MODE,
        .bus = &hfdcan1,
        .name = "right_ankle_pitch",
    },
    [ROBOT_JOINT_RIGHT_LEG_ANKLE_ROLL] = {
        .joint_id = ROBOT_JOINT_RIGHT_LEG_ANKLE_ROLL,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_RIGHT_LEG,
        .command_id = 0x08,
        .feedback_id = 0x18,
        .default_mode = MIT_MODE,
        .bus = &hfdcan1,
        .name = "right_ankle_roll",
    },
    [ROBOT_JOINT_RIGHT_LEG_TOE] = {
        .joint_id = ROBOT_JOINT_RIGHT_LEG_TOE,
        .model = ROBOT_MOTOR_DM4340,
        .limb = ROBOT_LIMB_RIGHT_LEG,
        .command_id = 0x09,
        .feedback_id = 0x19,
        .default_mode = MIT_MODE,
        .bus = &hfdcan1,
        .name = "right_toe",
    },
};

static RobotJointEntry g_joint_entries[ROBOT_JOINT_COUNT];
static RobotJointBusCache g_joint_bus_cache[ROBOT_JOINT_MAX_BUS_COUNT];
static uint32_t g_joint_bus_cache_count = 0U;
static bool g_joint_initialized = false;

/**
 * @brief Interpret the provided 32-bit word as an IEEE754 floating-point value.
 */
float Hex_To_Float(uint32_t *Byte,int num)
{
        (void)num;
        return *((float*)Byte);
}

uint32_t FloatTohex(float HEX)
{
        return *( uint32_t *)&HEX;
}

/**
 * @brief Convert a float into an unsigned integer limited to a given range.
 *
 * The helper rescales the floating point number into the interval defined by
 * @p x_min and @p x_max and maps it to an unsigned integer that fits within
 * @p bits bits.
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
        /* Converts a float to an unsigned int, given range and number of bits */
        float span = x_max - x_min;
        float offset = x_min;
        return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

static void RobotJointManager_ClearBusCache(void)
{
    for (uint32_t i = 0; i < ROBOT_JOINT_MAX_BUS_COUNT; i++)
    {
        g_joint_bus_cache[i].bus = NULL;
        memset(g_joint_bus_cache[i].feedback_lookup, 0, sizeof(g_joint_bus_cache[i].feedback_lookup));
    }
    g_joint_bus_cache_count = 0U;
}

static RobotJointBusCache *RobotJointManager_FindBusCache(hcan_t *bus)
{
    if (bus == NULL)
    {
        return NULL;
    }

    for (uint32_t i = 0; i < g_joint_bus_cache_count; i++)
    {
        if (g_joint_bus_cache[i].bus == bus)
        {
            return &g_joint_bus_cache[i];
        }
    }

    return NULL;
}

static RobotJointBusCache *RobotJointManager_GetOrCreateBusCache(hcan_t *bus)
{
    if (bus == NULL)
    {
        return NULL;
    }

    RobotJointBusCache *cache = RobotJointManager_FindBusCache(bus);
    if (cache != NULL)
    {
        return cache;
    }

    if (g_joint_bus_cache_count >= ROBOT_JOINT_MAX_BUS_COUNT)
    {
        return NULL;
    }

    cache = &g_joint_bus_cache[g_joint_bus_cache_count++];
    cache->bus = bus;
    memset(cache->feedback_lookup, 0, sizeof(cache->feedback_lookup));
    return cache;
}

static void RobotJointManager_RemoveEntryFromBusCache(RobotJointEntry *entry)
{
    if (entry == NULL || entry->config.bus == NULL)
    {
        return;
    }

    RobotJointBusCache *cache = RobotJointManager_FindBusCache(entry->config.bus);
    if (cache == NULL)
    {
        return;
    }

    if (entry->config.feedback_id < ROBOT_JOINT_FEEDBACK_LOOKUP_SIZE &&
        cache->feedback_lookup[entry->config.feedback_id] == entry)
    {
        cache->feedback_lookup[entry->config.feedback_id] = NULL;
    }
}

static void RobotJointManager_AddEntryToBusCache(RobotJointEntry *entry)
{
    if (entry == NULL || entry->config.bus == NULL)
    {
        return;
    }

    RobotJointBusCache *cache = RobotJointManager_GetOrCreateBusCache(entry->config.bus);
    if (cache == NULL)
    {
        return;
    }

    if (entry->config.feedback_id < ROBOT_JOINT_FEEDBACK_LOOKUP_SIZE)
    {
        cache->feedback_lookup[entry->config.feedback_id] = entry;
    }
}

static RobotJointEntry *RobotJointManager_LookupEntry(hcan_t *bus, uint16_t feedback_id)
{
    RobotJointBusCache *cache = RobotJointManager_FindBusCache(bus);
    if (cache == NULL || feedback_id >= ROBOT_JOINT_FEEDBACK_LOOKUP_SIZE)
    {
        return NULL;
    }

    return cache->feedback_lookup[feedback_id];
}
/**
 * @brief Convert an unsigned integer into a floating-point value.
 *
 * The helper performs the inverse mapping of @ref float_to_uint.
 * It scales the integer stored in @p x_int back into the range defined by
 * @p x_min and @p x_max.
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode)
{
  motor->mode=mode;
  motor->para.id=id;
}

void wheel_motor_init(Wheel_Motor_t *motor,uint16_t id,uint16_t mode)
{
  motor->mode=mode;
  motor->para.id=id;
}

/**
 * @brief Decode the standard feedback frame produced by a DM4310 actuator.
 *
 * The 8-byte MIT frame layout is common across most DM series motors. The
 * helper extracts the packed integers and stores both the raw words and the
 * converted engineering values inside @p motor.
 */
void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{
        if(data_len==FDCAN_DLC_BYTES_8)
        {// Expect the standard 8-byte MIT feedback frame.
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN1, P_MAX1, 16); // (-12.5,12.5)
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN1, V_MAX1, 12); // (-30.0,30.0)
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN1, T_MAX1, 12);  // (-10.0,10.0)
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}


void dm4340_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==FDCAN_DLC_BYTES_8)
	{// Expect the standard 8-byte MIT feedback frame.
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN2, P_MAX2, 16);
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN2, V_MAX2, 12); 
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN2, T_MAX2, 12); 
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}

void dm6006_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==FDCAN_DLC_BYTES_8)
	{// Expect the standard 8-byte MIT feedback frame.
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN3, P_MAX3, 16);
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN3, V_MAX3, 12); 
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN3, T_MAX3, 12); 
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}

void dm8006_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==FDCAN_DLC_BYTES_8)
	{// Expect the standard 8-byte MIT feedback frame.
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN4, P_MAX4, 16);
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN4, V_MAX4, 12); 
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN4, T_MAX4, 12); 
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}

void dm3507_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==FDCAN_DLC_BYTES_8)
	{// Expect the standard 8-byte MIT feedback frame.
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN5, P_MAX5, 16);
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN5, V_MAX5, 12); 
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN5, T_MAX5, 12); 
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}

void dm10010l_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==FDCAN_DLC_BYTES_8)
	{// Expect the standard 8-byte MIT feedback frame.
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN6, P_MAX6, 16);
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN6, V_MAX6, 12); 
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN6, T_MAX6, 12); 
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}

void dm6248p_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==FDCAN_DLC_BYTES_8)
	{// Expect the standard 8-byte MIT feedback frame.
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN7, P_MAX7, 16);
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN7, V_MAX7, 12); 
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN7, T_MAX7, 12); 
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}

// Convert the raw MIT cache words into floating-point values.
void dm4310_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data)
{ 
	motor->para.p_int_test=(rx_data[2]<<8)|rx_data[3];
	 
	motor->para.v_int_test=(((int16_t)rx_data[4])<<4)|((rx_data[5]&0xF0)>>4);
							
	motor->para.kp_int_test=(((int16_t)(rx_data[5]&0x0F))<<8)|rx_data[6];
							
	motor->para.kd_int_test=(((int16_t)rx_data[7])<<4)|((rx_data[8]&0xF0)>>4);
							
	motor->para.t_int_test=(((int16_t)(rx_data[8]&0x0F))<<8)|rx_data[9];
							
	motor->para.pos_set = uint_to_float(motor->para.p_int_test, P_MIN1, P_MAX1, 16);
	motor->para.vel_set = uint_to_float(motor->para.v_int_test, V_MIN1, V_MAX1, 12); 
	motor->para.tor_set = uint_to_float(motor->para.t_int_test, T_MIN1, T_MAX1, 12);
	motor->para.kp_test = uint_to_float(motor->para.kp_int_test, KP_MIN1, KP_MAX1, 12);
	motor->para.kd_test = uint_to_float(motor->para.kd_int_test, KD_MIN1, KD_MAX1, 12);
}

void dm4340_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data)
{ 
	motor->para.p_int_test=(rx_data[2]<<8)|rx_data[3];
	 
	motor->para.v_int_test=(((int16_t)rx_data[4])<<4)|((rx_data[5]&0xF0)>>4);
							
	motor->para.kp_int_test=(((int16_t)(rx_data[5]&0x0F))<<8)|rx_data[6];
							
	motor->para.kd_int_test=(((int16_t)rx_data[7])<<4)|((rx_data[8]&0xF0)>>4);
							
	motor->para.t_int_test=(((int16_t)(rx_data[8]&0x0F))<<8)|rx_data[9];
							
	motor->para.pos_set = uint_to_float(motor->para.p_int_test, P_MIN2, P_MAX2, 16);
	motor->para.vel_set = uint_to_float(motor->para.v_int_test, V_MIN2, V_MAX2, 12); 
	motor->para.tor_set = uint_to_float(motor->para.t_int_test, T_MIN2, T_MAX2, 12);
	motor->para.kp_test = uint_to_float(motor->para.kp_int_test, KP_MIN2, KP_MAX2, 12);
	motor->para.kd_test = uint_to_float(motor->para.kd_int_test, KD_MIN2, KD_MAX2, 12); 
}

void dm6006_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data)
{ 
	motor->para.p_int_test=(rx_data[2]<<8)|rx_data[3];
	 
	motor->para.v_int_test=(((int16_t)rx_data[4])<<4)|((rx_data[5]&0xF0)>>4);
							
	motor->para.kp_int_test=(((int16_t)(rx_data[5]&0x0F))<<8)|rx_data[6];
							
	motor->para.kd_int_test=(((int16_t)rx_data[7])<<4)|((rx_data[8]&0xF0)>>4);
							
	motor->para.t_int_test=(((int16_t)(rx_data[8]&0x0F))<<8)|rx_data[9];
							
	motor->para.pos_set = uint_to_float(motor->para.p_int_test, P_MIN3, P_MAX3, 16);
	motor->para.vel_set = uint_to_float(motor->para.v_int_test, V_MIN3, V_MAX3, 12); 
	motor->para.tor_set = uint_to_float(motor->para.t_int_test, T_MIN3, T_MAX3, 12);
	motor->para.kp_test = uint_to_float(motor->para.kp_int_test, KP_MIN3, KP_MAX3, 12);
	motor->para.kd_test = uint_to_float(motor->para.kd_int_test, KD_MIN3, KD_MAX3, 12);
}

void dm8006_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data)
{ 
	motor->para.p_int_test=(rx_data[2]<<8)|rx_data[3];
	 
	motor->para.v_int_test=(((int16_t)rx_data[4])<<4)|((rx_data[5]&0xF0)>>4);
							
	motor->para.kp_int_test=(((int16_t)(rx_data[5]&0x0F))<<8)|rx_data[6];
							
	motor->para.kd_int_test=(((int16_t)rx_data[7])<<4)|((rx_data[8]&0xF0)>>4);
							
	motor->para.t_int_test=(((int16_t)(rx_data[8]&0x0F))<<8)|rx_data[9];
							
	motor->para.pos_set = uint_to_float(motor->para.p_int_test, P_MIN4, P_MAX4, 16);
	motor->para.vel_set = uint_to_float(motor->para.v_int_test, V_MIN4, V_MAX4, 12); 
	motor->para.tor_set = uint_to_float(motor->para.t_int_test, T_MIN4, T_MAX4, 12);
	motor->para.kp_test = uint_to_float(motor->para.kp_int_test, KP_MIN4, KP_MAX4, 12);
	motor->para.kd_test = uint_to_float(motor->para.kd_int_test, KD_MIN4, KD_MAX4, 12);
}

void dm10010l_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data)
{ 
	motor->para.p_int_test=(rx_data[2]<<8)|rx_data[3];
	 
	motor->para.v_int_test=(((int16_t)rx_data[4])<<4)|((rx_data[5]&0xF0)>>4);
							
	motor->para.kp_int_test=(((int16_t)(rx_data[5]&0x0F))<<8)|rx_data[6];
							
	motor->para.kd_int_test=(((int16_t)rx_data[7])<<4)|((rx_data[8]&0xF0)>>4);
							
	motor->para.t_int_test=(((int16_t)(rx_data[8]&0x0F))<<8)|rx_data[9];
							
	motor->para.pos_set = uint_to_float(motor->para.p_int_test, P_MIN6, P_MAX6, 16);
	motor->para.vel_set = uint_to_float(motor->para.v_int_test, V_MIN6, V_MAX6, 12); 
	motor->para.tor_set = uint_to_float(motor->para.t_int_test, T_MIN6, T_MAX6, 12);
	motor->para.kp_test = uint_to_float(motor->para.kp_int_test, KP_MIN6, KP_MAX6, 12);
	motor->para.kd_test = uint_to_float(motor->para.kd_int_test, KD_MIN6, KD_MAX6, 12);
}

void dm6248p_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data)
{ 
	motor->para.p_int_test=(rx_data[2]<<8)|rx_data[3];
	 
	motor->para.v_int_test=(((int16_t)rx_data[4])<<4)|((rx_data[5]&0xF0)>>4);
							
	motor->para.kp_int_test=(((int16_t)(rx_data[5]&0x0F))<<8)|rx_data[6];
							
	motor->para.kd_int_test=(((int16_t)rx_data[7])<<4)|((rx_data[8]&0xF0)>>4);
							
	motor->para.t_int_test=(((int16_t)(rx_data[8]&0x0F))<<8)|rx_data[9];
							
	motor->para.pos_set = uint_to_float(motor->para.p_int_test, P_MIN7, P_MAX7, 16);
	motor->para.vel_set = uint_to_float(motor->para.v_int_test, V_MIN7, V_MAX7, 12); 
	motor->para.tor_set = uint_to_float(motor->para.t_int_test, T_MIN7, T_MAX7, 12);
	motor->para.kp_test = uint_to_float(motor->para.kp_int_test, KP_MIN7, KP_MAX7, 12);
	motor->para.kd_test = uint_to_float(motor->para.kd_int_test, KD_MIN7, KD_MAX7, 12);
}

void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_send_data(hcan, id, data, 8);
}

void save_motor_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	canx_send_data(hcan, id, data, 8);
}


/**
 * @brief Disable a motor operating mode.
 *
 * The driver sends the standard MIT command frame filled with 0xFF bytes
 * (apart from the checksum) to instruct the motor to enter an idle state.
 *
 * @param hcan     Target CAN bus handle.
 * @param motor_id Identifier of the motor on the bus.
 * @param mode_id  Mode selector that should be disabled.
 */
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_send_data(hcan, id, data, 8);
}

/**
 * @brief Send a MIT mode set-point to a motor.
 *
 * The command encodes position, velocity, stiffness, damping and torque
 * into the packed MIT frame used by the DM series actuators.
 *
 * @param hcan     Target CAN bus handle.
 * @param motor_id Identifier of the motor on the bus.
 * @param pos      Desired joint position in radians.
 * @param vel      Desired joint velocity in radians per second.
 * @param kp       Proportional gain.
 * @param kd       Derivative gain.
 * @param torq     Desired torque in newton metres.
 */
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN1,  P_MAX1,  16);
	vel_tmp = float_to_uint(vel,  V_MIN1,  V_MAX1,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN1, KP_MAX1, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN1, KD_MAX1, 12);
	tor_tmp = float_to_uint(torq, T_MIN1,  T_MAX1,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}
/**
 * @brief Send a pure velocity command to the motor.
 *
 * The frame sets the target speed while leaving the position-related
 * registers untouched, which is useful for wheel or continuous joints.
 *
 * @param hcan     Target CAN bus handle.
 * @param motor_id Identifier of the motor on the bus.
 * @param vel      Desired velocity in radians per second.
 */
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 4);
}


//4340
void mit_ctrl2(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN2,  P_MAX2,  16);
	vel_tmp = float_to_uint(vel,  V_MIN2,  V_MAX2,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN2, KP_MAX2, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN2, KD_MAX2, 12);
	tor_tmp = float_to_uint(torq, T_MIN2,  T_MAX2,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}


//6006
void mit_ctrl3(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN3,  P_MAX3,  16);
	vel_tmp = float_to_uint(vel,  V_MIN3,  V_MAX3,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN3, KP_MAX3, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN3, KD_MAX3, 12);
	tor_tmp = float_to_uint(torq, T_MIN3,  T_MAX3,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}

//8006
void mit_ctrl4(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN4,  P_MAX4,  16);
	vel_tmp = float_to_uint(vel,  V_MIN4,  V_MAX4,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN4, KP_MAX4, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN4, KD_MAX4, 12);
	tor_tmp = float_to_uint(torq, T_MIN4,  T_MAX4,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}

//3507
void mit_ctrl5(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN5,  P_MAX5,  16);
	vel_tmp = float_to_uint(vel,  V_MIN5,  V_MAX5,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN5, KP_MAX5, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN5, KD_MAX5, 12);
	tor_tmp = float_to_uint(torq, T_MIN5,  T_MAX5,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}


void mit_ctrl_test(hcan_t* hcan, uint16_t motor_id,Joint_Motor_t *motor)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = motor->para.p_int_test;
	vel_tmp = motor->para.v_int_test;
	kp_tmp  = motor->para.kp_int_test;
	kd_tmp  = motor->para.kd_int_test;
	tor_tmp = motor->para.t_int_test;

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}

// Initialise the cached command words to zero so the driver starts from a neutral state.
void dm4310_fbdata_init(Joint_Motor_t *motor)
{
  motor->para.p_int_test = float_to_uint(0.0f,  P_MIN1,  P_MAX1,  16);
	motor->para.v_int_test = float_to_uint(0.0f,  V_MIN1,  V_MAX1,  12);
	motor->para.kp_int_test  = float_to_uint(0.0f,   KP_MIN1, KP_MAX1, 12);
	motor->para.kd_int_test  = float_to_uint(0.0f,   KD_MIN1, KD_MAX1, 12);
	motor->para.t_int_test = float_to_uint(0.0f, T_MIN1,  T_MAX1,  12);
}


void dm4340_fbdata_init(Joint_Motor_t *motor)
{
  motor->para.p_int_test = float_to_uint(0.0f,  P_MIN2,  P_MAX2,  16);
	motor->para.v_int_test = float_to_uint(0.0f,  V_MIN2,  V_MAX2,  12);
	motor->para.kp_int_test  = float_to_uint(0.0f,   KP_MIN2, KP_MAX2, 12);
	motor->para.kd_int_test  = float_to_uint(0.0f,   KD_MIN2, KD_MAX2, 12);
	motor->para.t_int_test = float_to_uint(0.0f, T_MIN2,  T_MAX2,  12);
}

void dm6006_fbdata_init(Joint_Motor_t *motor)
{
  motor->para.p_int_test = float_to_uint(0.0f,  P_MIN3,  P_MAX3,  16);
	motor->para.v_int_test = float_to_uint(0.0f,  V_MIN3,  V_MAX3,  12);
	motor->para.kp_int_test  = float_to_uint(0.0f,   KP_MIN3, KP_MAX3, 12);
	motor->para.kd_int_test  = float_to_uint(0.0f,   KD_MIN3, KD_MAX3, 12);
	motor->para.t_int_test = float_to_uint(0.0f, T_MIN3,  T_MAX3,  12);
}

void dm8006_fbdata_init(Joint_Motor_t *motor)
{
    motor->para.p_int_test = float_to_uint(0.0f, P_MIN4, P_MAX4, 16);
    motor->para.v_int_test = float_to_uint(0.0f, V_MIN4, V_MAX4, 12);
    motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN4, KP_MAX4, 12);
    motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN4, KD_MAX4, 12);
    motor->para.t_int_test = float_to_uint(0.0f, T_MIN4, T_MAX4, 12);
}

void dm3507_fbdata_init(Joint_Motor_t *motor)
{
    motor->para.p_int_test = float_to_uint(0.0f, P_MIN5, P_MAX5, 16);
    motor->para.v_int_test = float_to_uint(0.0f, V_MIN5, V_MAX5, 12);
    motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN5, KP_MAX5, 12);
    motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN5, KD_MAX5, 12);
    motor->para.t_int_test = float_to_uint(0.0f, T_MIN5, T_MAX5, 12);
}

void dm10010l_fbdata_init(Joint_Motor_t *motor)
{
    motor->para.p_int_test = float_to_uint(0.0f, P_MIN6, P_MAX6, 16);
    motor->para.v_int_test = float_to_uint(0.0f, V_MIN6, V_MAX6, 12);
    motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN6, KP_MAX6, 12);
    motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN6, KD_MAX6, 12);
    motor->para.t_int_test = float_to_uint(0.0f, T_MIN6, T_MAX6, 12);
}

void dm6248p_fbdata_init(Joint_Motor_t *motor)
{
    motor->para.p_int_test = float_to_uint(0.0f, P_MIN7, P_MAX7, 16);
    motor->para.v_int_test = float_to_uint(0.0f, V_MIN7, V_MAX7, 12);
    motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN7, KP_MAX7, 12);
    motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN7, KD_MAX7, 12);
    motor->para.t_int_test = float_to_uint(0.0f, T_MIN7, T_MAX7, 12);
}

static void RobotJointManager_EnsureInit(void)
{
    if (!g_joint_initialized)
    {
        memset(g_joint_entries, 0, sizeof(g_joint_entries));
        RobotJointManager_ClearBusCache();
        g_joint_initialized = true;
    }
}

const RobotJointHardwareConfig *RobotJointHardware_GetConfig(RobotJointId joint_id)
{
    if (joint_id >= ROBOT_JOINT_COUNT)
    {
        return NULL;
    }
    return &g_joint_hw_table[joint_id];
}

bool RobotJointHardware_SetConfig(const RobotJointHardwareConfig *config)
{
    if (config == NULL || config->joint_id >= ROBOT_JOINT_COUNT)
    {
        return false;
    }

    RobotJointHardwareConfig *target = &g_joint_hw_table[config->joint_id];
    *target = *config;
    if (target->default_mode == 0U)
    {
        target->default_mode = MIT_MODE;
    }
    return true;
}

static void RobotJointManager_InitFeedbackCache(const RobotJointConfig *config)
{
    if (config == NULL || config->joint == NULL)
    {
        return;
    }

    switch (config->model)
    {
    case ROBOT_MOTOR_DM4310:
        dm4310_fbdata_init(config->joint);
        break;
    case ROBOT_MOTOR_DM4340:
    case ROBOT_MOTOR_DM6248P:
        dm4340_fbdata_init(config->joint);
        break;
    case ROBOT_MOTOR_DM6006:
        dm6006_fbdata_init(config->joint);
        break;
    case ROBOT_MOTOR_DM8006:
        dm8006_fbdata_init(config->joint);
        break;
    case ROBOT_MOTOR_DM3507:
        dm3507_fbdata_init(config->joint);
        break;
    case ROBOT_MOTOR_DM10010L:
        dm10010l_fbdata_init(config->joint);
        break;
    default:
        break;
    }
}

static void RobotJointManager_HandleFeedbackForEntry(const RobotJointConfig *config, uint8_t *data, uint32_t len)
{
    if (config == NULL || config->joint == NULL || data == NULL)
    {
        return;
    }

    switch (config->model)
    {
    case ROBOT_MOTOR_DM4310:
        dm4310_fbdata(config->joint, data, len);
        break;
    case ROBOT_MOTOR_DM4340:
    case ROBOT_MOTOR_DM6248P:
        dm4340_fbdata(config->joint, data, len);
        break;
    case ROBOT_MOTOR_DM6006:
        dm6006_fbdata(config->joint, data, len);
        break;
    case ROBOT_MOTOR_DM8006:
        dm8006_fbdata(config->joint, data, len);
        break;
    case ROBOT_MOTOR_DM3507:
        dm3507_fbdata(config->joint, data, len);
        break;
    case ROBOT_MOTOR_DM10010L:
        dm10010l_fbdata(config->joint, data, len);
        break;
    default:
        break;
    }
}

static void RobotJointManager_SendMitInternal(const RobotJointConfig *config, float pos, float vel, float kp, float kd, float torq)
{
    if (config == NULL || config->bus == NULL)
    {
        return;
    }

    switch (config->model)
    {
    case ROBOT_MOTOR_DM4310:
        mit_ctrl(config->bus, config->command_id, pos, vel, kp, kd, torq);
        break;
    case ROBOT_MOTOR_DM4340:
    case ROBOT_MOTOR_DM6248P:
        mit_ctrl2(config->bus, config->command_id, pos, vel, kp, kd, torq);
        break;
    case ROBOT_MOTOR_DM6006:
        mit_ctrl3(config->bus, config->command_id, pos, vel, kp, kd, torq);
        break;
    case ROBOT_MOTOR_DM8006:
        mit_ctrl4(config->bus, config->command_id, pos, vel, kp, kd, torq);
        break;
    case ROBOT_MOTOR_DM3507:
        mit_ctrl5(config->bus, config->command_id, pos, vel, kp, kd, torq);
        break;
    case ROBOT_MOTOR_DM10010L:
        mit_ctrl4(config->bus, config->command_id, pos, vel, kp, kd, torq);
        break;
    default:
        mit_ctrl(config->bus, config->command_id, pos, vel, kp, kd, torq);
        break;
    }
}

void RobotJointManager_Reset(void)
{
    memset(g_joint_entries, 0, sizeof(g_joint_entries));
    RobotJointManager_ClearBusCache();
    g_joint_initialized = true;
}

bool RobotJointManager_Register(const RobotJointConfig *config)
{
    if (config == NULL || config->joint_id >= ROBOT_JOINT_COUNT || config->joint == NULL ||
        config->bus == NULL || config->command_id == 0U)
    {
        return false;
    }

    RobotJointManager_EnsureInit();

    RobotJointEntry *entry = &g_joint_entries[config->joint_id];
    if (entry->registered)
    {
        RobotJointManager_RemoveEntryFromBusCache(entry);
    }
    entry->config = *config;

    if (entry->config.mode == 0U)
    {
        entry->config.mode = MIT_MODE;
    }
    if (entry->config.feedback_id == 0U)
    {
        entry->config.feedback_id = entry->config.command_id + ROBOT_JOINT_FEEDBACK_OFFSET;
    }
    if (entry->config.model == ROBOT_MOTOR_UNKNOWN)
    {
        const RobotJointHardwareConfig *hw = RobotJointHardware_GetConfig(config->joint_id);
        entry->config.model = hw != NULL ? hw->model : ROBOT_MOTOR_DM4310;
    }
    if (entry->config.name == NULL)
    {
        const RobotJointHardwareConfig *hw = RobotJointHardware_GetConfig(config->joint_id);
        entry->config.name = (hw != NULL && hw->name != NULL) ? hw->name : "";
    }

    joint_motor_init(entry->config.joint, entry->config.command_id, entry->config.mode);
    RobotJointManager_InitFeedbackCache(&entry->config);

    RobotJointManager_AddEntryToBusCache(entry);
    entry->registered = true;
    return true;
}

bool RobotJointManager_RegisterJoint(RobotJointId joint_id, Joint_Motor_t *joint, uint16_t override_mode)
{
    const RobotJointHardwareConfig *hw = RobotJointHardware_GetConfig(joint_id);
    if (hw == NULL || hw->bus == NULL || hw->command_id == 0U)
    {
        return false;
    }

    RobotJointConfig config = {
        .joint_id = joint_id,
        .joint = joint,
        .model = hw->model,
        .limb = hw->limb,
        .bus = hw->bus,
        .command_id = hw->command_id,
        .feedback_id = hw->feedback_id,
        .mode = (override_mode != 0U) ? override_mode : hw->default_mode,
        .name = hw->name,
    };

    return RobotJointManager_Register(&config);
}

Joint_Motor_t *RobotJointManager_GetMotor(RobotJointId joint_id)
{
    RobotJointManager_EnsureInit();
    if (joint_id >= ROBOT_JOINT_COUNT)
    {
        return NULL;
    }
    RobotJointEntry *entry = &g_joint_entries[joint_id];
    if (!entry->registered)
    {
        return NULL;
    }
    return entry->config.joint;
}

bool RobotJointManager_SendMIT(RobotJointId joint_id, float pos, float vel, float kp, float kd, float torq)
{
    RobotJointManager_EnsureInit();
    if (joint_id >= ROBOT_JOINT_COUNT)
    {
        return false;
    }

    RobotJointEntry *entry = &g_joint_entries[joint_id];
    if (!entry->registered)
    {
        return false;
    }

    RobotJointManager_SendMitInternal(&entry->config, pos, vel, kp, kd, torq);
    return true;
}

bool RobotJointManager_SendMITUsingCache(RobotJointId joint_id)
{
    RobotJointManager_EnsureInit();
    if (joint_id >= ROBOT_JOINT_COUNT)
    {
        return false;
    }

    RobotJointEntry *entry = &g_joint_entries[joint_id];
    if (!entry->registered || entry->config.joint == NULL)
    {
        return false;
    }

    mit_ctrl_test(entry->config.bus, entry->config.command_id, entry->config.joint);
    return true;
}

bool RobotJointManager_SaveZero(RobotJointId joint_id)
{
    RobotJointManager_EnsureInit();
    if (joint_id >= ROBOT_JOINT_COUNT)
    {
        return false;
    }

    RobotJointEntry *entry = &g_joint_entries[joint_id];
    if (!entry->registered)
    {
        return false;
    }

    save_motor_zero(entry->config.bus, entry->config.command_id, entry->config.mode);
    return true;
}

void RobotJointManager_EnableLimb(RobotLimb limb, uint8_t repeat, uint16_t delay_ms)
{
    RobotJointManager_EnsureInit();
    if (limb >= ROBOT_LIMB_COUNT)
    {
        return;
    }

    if (repeat == 0U)
    {
        repeat = 1U;
    }

    for (uint8_t r = 0; r < repeat; r++)
    {
        for (uint32_t i = 0; i < ROBOT_JOINT_COUNT; i++)
        {
            RobotJointEntry *entry = &g_joint_entries[i];
            if (entry->registered && entry->config.limb == limb)
            {
                enable_motor_mode(entry->config.bus, entry->config.command_id, entry->config.mode);
            }
        }
        if (delay_ms > 0U)
        {
            osDelay(delay_ms);
        }
    }
}

void RobotJointManager_EnableAll(uint8_t repeat, uint16_t delay_ms)
{
    for (RobotLimb limb = ROBOT_LIMB_WAIST; limb < ROBOT_LIMB_COUNT; limb = (RobotLimb)(limb + 1))
    {
        RobotJointManager_EnableLimb(limb, repeat, delay_ms);
    }
}

void RobotJointManager_HandleFeedback(hcan_t *bus, uint16_t feedback_id, uint8_t *data, uint32_t len)
{
    RobotJointManager_EnsureInit();
    if (bus == NULL)
    {
        return;
    }

    RobotJointEntry *entry = RobotJointManager_LookupEntry(bus, feedback_id);
    if (entry != NULL && entry->registered)
    {
        RobotJointManager_HandleFeedbackForEntry(&entry->config, data, len);
        return;
    }

    for (uint32_t i = 0; i < ROBOT_JOINT_COUNT; i++)
    {
        RobotJointEntry *candidate = &g_joint_entries[i];
        if (candidate->registered && candidate->config.bus == bus && candidate->config.feedback_id == feedback_id)
        {
            RobotJointManager_HandleFeedbackForEntry(&candidate->config, data, len);
            RobotJointManager_AddEntryToBusCache(candidate);
            break;
        }
    }
}
