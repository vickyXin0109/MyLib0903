/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "MyLib.hpp"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

using namespace std;
using namespace hardware;

class Custom
{
public:
  
  int CANSend();
  void CANRecv();
  void RobotControl();

  //Safety safe;
  Custom() : can_control("can1")
  {}

  CanControl can_control;

  float qInit[MOTOR_NUMBER] = {0};
  float qDes[MOTOR_NUMBER] = {0};
  float sin_mid_q[MOTOR_NUMBER] = {1.2, 1.2, -2.0};
  float Kp[MOTOR_NUMBER] = {0};
  float Kd[MOTOR_NUMBER] = {0};
  double time_consume = 0;
  int rate_count = 0;
  int sin_count = 0;
  int motiontime = 0;
  float dt = 0.002; // 0.001~0.01
};

int Custom::CANSend()
{
  int ret = 0;
  ret = can_control.Safety_PositionProtect();
  if (ret == 0 )
  {
    can_control.SendMotorsCMD();
    return 0;
  }
  else
    return -1;
}

void Custom::CANRecv()
{
  can_control.RecvMotorsDATA();
}


double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
  double p;
  rate = std::min(std::max(rate, 0.0), 1.0);
  p = initPos * (1 - rate) + targetPos * rate;
  return p;
}

void Custom::RobotControl()
{
  motiontime++;
  can_control.RecvMotorsDATA();
  printf("[io] :motiontime : %d\n", motiontime);
  printf("[DATA]: p= %f; v= %f; t= %f; temp= %f; error= %d",
    can_control.motorsData[0]->position_, can_control.motorsData[0]->velocity_, 
    can_control.motorsData[0]->torque_, can_control.motorsData[0]->temp_, 
    can_control.motorsData[0]->error_);

  can_control.motorsCmd[FR_0]->torque_ = +1.0f;

  if (motiontime >= 0)
  {
    if (motiontime >= 0 && motiontime < 10)
        can_control.SetMotorsHome();

    if (motiontime >= 10 && motiontime < 400)
    {
      rate_count++;
      double rate = rate_count / 200.0; // needs count to 200
      Kp[0] = 30.0;
      Kp[1] = 30.0;
      Kp[2] = 30.0;
      Kd[0] = 5.0;
      Kd[1] = 5.0;
      Kd[2] = 5.0;

      qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
      qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
      qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
    }
    double sin_joint1, sin_joint2;
    if (motiontime >= 400)
    {
      sin_count++;
      sin_joint1 = 0.6 * sin(3 * M_PI * sin_count / 1000.0);
      sin_joint2 = -0.6 * sin(1.8 * M_PI * sin_count / 1000.0);
      qDes[0] = sin_mid_q[0];
      qDes[1] = sin_mid_q[1];
      qDes[2] = sin_mid_q[2] + sin_joint2;
    }

    can_control.motorsCmd[FR_0]->position_ = qDes[0];
    can_control.motorsCmd[FR_0]->velocity_ = 0;
    can_control.motorsCmd[FR_0]->kp_ = Kp[0];
    can_control.motorsCmd[FR_0]->kd_ = Kd[0];
    can_control.motorsCmd[FR_0]->torque_ = +1.0f;

    can_control.motorsCmd[FR_1]->position_ = qDes[1];
    can_control.motorsCmd[FR_1]->velocity_ = 0;
    can_control.motorsCmd[FR_1]->kp_ = Kp[1];
    can_control.motorsCmd[FR_1]->kd_ = Kd[1];
    can_control.motorsCmd[FR_1]->torque_ = +1.0f;

    can_control.motorsCmd[FR_2]->position_ = qDes[2];
    can_control.motorsCmd[FR_2]->velocity_ = 0;
    can_control.motorsCmd[FR_2]->kp_ = Kp[2];
    can_control.motorsCmd[FR_2]->kd_ = Kd[2];
    can_control.motorsCmd[FR_2]->torque_ = +1.0f;
  }
}

int main(void)
{
  std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "NOTE: The robot also needs to be set to LOW-level mode, otherwise it will make strange noises and this example will not run successfully! " << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom;
    while(1)
    {
        custom.RobotControl();
        if (custom.motiontime >= 10)
        {
            int ret = custom.CANSend();
            if (ret == -1 )
            {
              break;
            }   
            custom.CANRecv();       
        }
        
        if (custom.motiontime >500)
        {
            //custom.can_control.DisableMotors();
            break;
        }
    }

  return 0;
}
