#ifndef __MY_LIB_H__
#define __MY_LIB_H__

#include <signal.h>
#include <unistd.h>
#include "comm.h"
#include "quadruped.h"


namespace hardware{

class CanControl{
    public:
        CanControl(const char *can_name);
        ~CanControl();

        volatile sig_atomic_t break_flag = 0;
        DrMotorCan *can;
        MotorCMD *motorsCmd[MOTOR_NUMBER];
        MotorDATA *motorsData[MOTOR_NUMBER];
        
        //void InitMotors();
        void GetStatus();
        //void InitCmdData();
        //void SetSend(MotorCMD *set_cmd);
        void SendMotorsCMD();
        void RecvMotorsDATA();
        void SetMotorsHome();

        void EnableMotros();
        void DisableMotors();

        int Safety_PositionProtect();

    private:

        MotorDATA *motor_data;
        void SetCMDs(int function);
        void DisDatas();
};
}

#endif