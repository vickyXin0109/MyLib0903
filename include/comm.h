#ifndef __MY_LIB_COMM_H__
#define __MY_LIB_COMM_H__

#include <stdint.h>

#define MOTOR_NUMBER 3

#define HIP_MAX 1.57f   //ID=1
#define HIP_MIN -1.57f  
#define THIGH_MAX 3.14f     //ID=2
#define THIGH_MIN -3.14f
#define CALF_MAX 3.14f  //ID=3
#define CALF_MIN -3.14f

namespace hardware {
//存储电机返回的数据
//Struct saving data from motor
typedef struct
{
    uint8_t motor_id_;
    uint8_t cmd_;
    float position_;
    float velocity_;
    float torque_;
    bool flag_;
    float temp_;
    uint16_t error_;//for motors update
    uint8_t recv_error_;//for recv error
    uint8_t send_error_;//for send error
}MotorDATA;

//存储发向电机的数据
//Struct of cmd sending to motor
typedef struct
{
    uint8_t motor_id_;
    uint8_t cmd_;
    float position_;
    float velocity_;
    float torque_;
    float kp_;
    float kd_;
}MotorCMD;

//DrMotorCan类，用于保存can的相关配置和资源
//DrMotorCan struct, saving can configs and resources
typedef struct{
    bool is_show_log_;
    int can_socket_;
    int epoll_fd_;
    pthread_mutex_t rw_mutex;
}DrMotorCan;
}
#endif