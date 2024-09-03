#pragma once

#include <stdint.h>

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
    uint16_t error_;
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