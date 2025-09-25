/**
  *********************************************************************
  * @file      VBUS_Check_task.c/h
  * @brief     Monitor the DC bus voltage and shut down the actuators when it drops below 22 V.
  * @note
  * @history
  *
  @verbatim
  ===============================================================================

  ===============================================================================
  @endverbatim
  *********************************************************************
  */
#include "vbus_check.h"
#include "adc.h"
#include "cmsis_os.h"
#include "gpio.h"

#include "control_board_profile.h"
#include "dm4310_drv.h"
#include "fdcan.h"
#include "tim.h"

static uint16_t adc_val[2];
static float vbus;


static uint16_t calibration_value=378;

uint8_t loss_voltage = 0;

#define vbus_threhold_disable (22.2f)

#define vbus_threhold_call (22.6f)

static void VBUS_DisableJoint(RobotJointId joint)
{
        Joint_Motor_t *motor = RobotJointManager_GetMotor(joint);
        const RobotJointHardwareConfig *hw = RobotJointHardware_GetConfig(joint);
        if(motor == NULL || hw == NULL || hw->bus == NULL)
        {
                return;
        }
        disable_motor_mode(hw->bus, motor->para.id, motor->mode);
        osDelay(5);
}

static void VBUS_DisableProfileJoints(const ControlBoardProfile *profile)
{
        for(size_t i = 0; i < profile->telemetry_joint_count; ++i)
        {
                VBUS_DisableJoint(profile->telemetry_joints[i]);
        }

        for(size_t i = 0; i < profile->body_joint_count; ++i)
        {
                VBUS_DisableJoint(profile->body_joints[i].joint);
        }
}

void VBUS_Check_task(void)
{

        HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val,2);

        const ControlBoardProfile *profile = ControlBoardProfile_GetActive();

        while(1)
        {
                vbus = ((adc_val[1]+calibration_value)*3.3f/65535)*11.0f;

                if(6.0f<vbus&&vbus<vbus_threhold_call)
                {// Sound the buzzer when the voltage approaches the warning threshold (22.6 V).
                        Buzzer_ON;
                }
                else
                {
                        Buzzer_OFF;
                }
                if(6.0f<vbus&&vbus<vbus_threhold_disable)
                {// Below 22.2 V the robot can no longer operate safely; shut everything down.
                        loss_voltage=1;
                        Power_OUT2_OFF;
                        Power_OUT1_OFF;

                        VBUS_DisableProfileJoints(profile);
                }
                else
                {
                        Power_OUT2_ON;
                        Power_OUT1_ON;

                        loss_voltage=0;
                }

                osDelay(100);
        }
}
