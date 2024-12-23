//
// Created by CYK on 2024/11/27.
//

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C"
{
#endif

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

//变结构pi调节
struct variable_structure_pid_param_t
{
    float Ap;
    float Bp;
    float Cp;
    float Ai;
    float Ci;
    float kd;
    float input_max_err;
    float max_out;
    float integral_higher_limit;
    float integral_lower_limit;
};

#ifdef __cplusplus
}
#endif

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

    friend class Chassis_Device;
};

class variable_structure_pid
{
protected:
    bool enable_flag;

    float set;
    float get;

    float error;
    float last_error;

    float pout;
    float iout;
    float dout;

    float kp;
    float ki;

public:
    variable_structure_pid_param_t param;
    float out;

    void Init(float max_out, float integral_limit, float Ap, float Bp, float Cp,float Ai,float Ci,float kd);
    float Calculate(float set, float get);

    friend class Chassis_Device;
};

#endif //PID_H
