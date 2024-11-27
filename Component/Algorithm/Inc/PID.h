//
// Created by CYK on 2024/11/27.
//

#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"

typedef struct
{
    float p;
    float i;
    float d;
    float max_out;
    float integral_higher_limit;
    float integral_lower_limit;
}pid_param;


typedef struct
{
    float max_out;
    float integral_limit;
    float kp;
    float ki;
    float kd;
}pid_init_param_t;

class pid
{
public:
    pid_param param;

    __RAM_FUNC void Init(float kp,float ki,float kd,float integral_limit,float max_out);
    float Calculate(float set, float get);
protected:
    bool enable_flag;

    float error;
    float last_error;
    float penultimate_error;

    float pout;
    float iout;
    float dout;
    float out;
};

#endif //PID_H
