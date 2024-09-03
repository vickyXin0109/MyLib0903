//is new!
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
    
    	CanControl::~CanControl(){
            DisableMotors();

            //回收资源
            //Reclaim allocated memory
            DrMotorCanDestroy(can);
        
        
            for(int i = 0; i < MOTOR_NUMBER; i++)
            {
                MotorCMDDestroy(motorsCmd[i]);
                MotorDATADestroy(motorsData[i]);
            }
        }

    void CanControl::EnableMotros()
    {
        SetCMDs(ENABLE_MOTOR);
    }

    void CanControl::DisableMotors()
    {
        //失能同一总线上的所有关节
        //Disable all motors on the can bus
        SetCMDs(DISABLE_MOTOR);
    }

    void CanControl::GetStatus()
    {
        SetCMDs(GET_STATUS_WORD);
    }

    void CanControl::SetMotorsHome()
    {
        SetCMDs(SET_HOME);
        printf("[INFO] datas are sended\r\n");
    }

    void CanControl::SendMotorsCMD()
    {
        SetCMDs(CONTROL_MOTOR);
        printf("[INFO] datas are sended\r\n");
    }

    void CanControl::SetCMDs(int function)
    {
        for(int i = 0; i < MOTOR_NUMBER; i++)
        {
            int motor_id = i+1;
            if (function == CONTROL_MOTOR) 
            {
                SetMotionCMD(motorsCmd[i], motor_id, CONTROL_MOTOR, motorsCmd[i]->position_, motorsCmd[i]->velocity_, motorsCmd[i]->torque_, motorsCmd[i]->kp_, motorsCmd[i]->kd_);
            }
            else
            {
                SetNormalCMD(motorsCmd[i], motor_id, function);
            }

            motorsData[i]->send_error_ = SendMsg(can, motorsCmd[i]);

            CheckSendRecvError(motor_id, motorsData[i]->send_error_);//just for print
        }
    }

    void CanControl::RecvMotorsDATA()
    {
        for(int i = 0; i < MOTOR_NUMBER; i++)
        {
            int motor_id = i+1;
            int err_recv;
            err_recv = RecvMsg(can, motor_data);
            motor_data->recv_error_ = err_recv;
            CheckSendRecvError(motor_id, motor_data->recv_error_);//just for print
            CheckMotorError(motor_id, motor_data->error_);//just for print
            DisDatas();
        }
        
    }

    void CanControl::DisDatas()
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
            motorsData[motor_data_arry]->position_ = motor_data->position_;
            motorsData[motor_data_arry]->velocity_ = motor_data->velocity_;
            motorsData[motor_data_arry]->torque_ = motor_data->torque_;
            motorsData[motor_data_arry]->temp_ = motor_data->temp_;
            motorsData[motor_data_arry]->error_ = motor_data->error_;
		}
    }

    int CanControl:: Safety_PositionProtect()
    {
        if (motorsCmd[FR_0]->position_ > HIP_MAX || motorsCmd[FR_0]->position_ < HIP_MIN)
        {
            printf("[WARN] FR_0 Motor out of soft-limit /n");
            return -1;
        }

        else if (motorsCmd[FR_1]->position_ > THIGH_MAX || motorsCmd[FR_1]->position_ < THIGH_MIN)
        {
            printf("[WARN] FR_1 Motor out of soft-limit /n");
            return -1;
        }

        else if (motorsCmd[FR_2]->position_ > CALF_MAX || motorsCmd[FR_2]->position_ < CALF_MIN)
        {
            printf("[WARN] FR_2 Motor out of soft-limit /n");
            return -1;
        }

        else
        {
            return 0;
        }


    }
}
