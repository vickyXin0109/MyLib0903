
#include "MyLib.hpp"
#include "deep_motor_sdk.h"

using namespace std;

namespace hardware{

    CanControl::CanControl(const char *can_name)
    {
       //创建基于socketcan的can0设备对象
        //Create an socketcan-based can0 device object 
        for(int i = 0; i < MOTOR_NUMBER; i++)
        {
            can = DrMotorCanCreate(can_name, true);
            motorsCmd[i] = MotorCMDCreate();
            motorsData[i] = MotorDATACreate(); 
        }
        motor_data = MotorDATACreate(); 

        EnableMotros();
    }
    
    	CanControl::~CanControl()
	{
        DisableMotors();
        //回收资源
        //Reclaim allocated memory
        DrMotorCanDestroy(can);
        for(int i = 0; i < MOTOR_NUMBER; i++)
        {
            MotorCMDDestroy(motorsCmd[i]);
            MotorDATADestroy(motorsData[i]);
        }
        printf("[INFO] Ended multi motor control\r\n");
	}

    void CanControl::EnableMotros()
    {
        SetMotorsCmd(ENABLE_MOTOR);
    }

    void CanControl::DisableMotors()
    {
        //失能同一总线上的所有关节
        //Disable all motors on the can bus
        SetMotorsCmd(DISABLE_MOTOR);
    }

    void CanControl::GetStatus()
    {
        SetMotorsCmd(GET_STATUS_WORD);
    }

    void CanControl::SetMotorsHome()
    {
        SetMotorsCmd(SET_HOME);
        printf("[INFO] datas are sended\r\n");
    }

    void CanControl::SendRecvMotorsData()
    {
        SetMotorsCmd(CONTROL_MOTOR);
        printf("[INFO] datas are sended\r\n");
    }

    void CanControl::SetMotorsCmd(int function)
    {
        for(int i = 0; i < MOTOR_NUMBER; i++)
        {
            int ret = 0;
            int motor_id = i+1;
            if (function == CONTROL_MOTOR) 
            {
                SetMotionCMD(motorsCmd[i], motor_id, CONTROL_MOTOR, motorsCmd[i]->position_, motorsCmd[i]->velocity_, motorsCmd[i]->torque_, motorsCmd[i]->kp_, motorsCmd[i]->kd_);
            }
            else
            {
                SetNormalCMD(motorsCmd[i], motor_id, function);
            }

            ret = SendRecv(can, motorsCmd[i], motor_data);
            RecvMotorData();
            CheckSendRecvError(motor_id, ret);
            CheckMotorError(motor_id, motor_data->error_);
        }
    }

    void CanControl::RecvMotorData()
    {
        uint8_t motor_id_fb = motor_data->motor_id_;
        uint8_t motor_data_arry = motor_id_fb - 1;
		uint8_t cmd = motor_data->cmd_;
        if (motor_data_arry < 0)
        {
            printf("[ERROR]: Motor_Id: %d is wrong!", motor_id_fb);
        }

        if (cmd == CONTROL_MOTOR)
		{
            motorsData[motor_data_arry] = motor_data;
		}

    }
}
