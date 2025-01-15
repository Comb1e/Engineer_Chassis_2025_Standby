//
// Created by CYK on 2024/11/27.
//

#include "PID.h"
#include <math.h>
#include "User_Lib.h"

__RAM_FUNC void pid::Init(float kp, float ki, float kd, float max_out,float i_limit_k,bool is_vel_control)
{
    this->param.p = kp;
    this->param.i = ki;
    this->param.d = kd;
    this->param.max_out = max_out;

    this->pout = 0;
    this->iout = 0;
    this->dout = 0;
    this->error = 0;
    this->last_error = 0;
    this->penultimate_error = 0;
    this->enable_flag = true;
    this->out = 0;
    this->i_limit_k = i_limit_k;
    this->is_vel_control = is_vel_control;
}

float pid::Calculate(float set, float get)
{
    this->error = set - get;

    this->pout = this->param.p * this->error;
    this->iout += this->param.i * this->error;
    this->dout = this->param.d * (this->error - this->last_error);

    if(this->is_vel_control)
    {
        this->Update_Integral_Limit(this->i_limit_k,set);
    }
    else
    {
        this->Update_Integral_Limit(this->i_limit_k,this->error);
    }
    VAL_LIMIT(this->iout,this->param.integral_lower_limit,this->param.integral_higher_limit);
    this->out = this->pout + this->iout + this->dout;
    ABS_LIMIT(this->out,this->param.max_out);
    this->last_error = this->error;

    if(!this->enable_flag)
    {
        this->out = 0;
    }
    return (this->out);
}

void pid::Update_Integral_Limit(float k,float set)
{
    float temp = ABS(k * set);
    VAL_LIMIT(temp,0.0f,0.7f);
    this->param.integral_higher_limit = temp;
    this->param.integral_lower_limit = -temp;
}


void variable_structure_pid::Init(float max_out,
                                  float integral_limit,
                                  float Ap,
                                  float Bp,
                                  float Cp,
                                  float Ai,
                                  float Ci,
                                  float kd)
{
    this->enable_flag = true;
    this->out = 0;
    this->iout = 0;
    this->dout = 0;
    this->pout = 0;
    this->error = 0;
    this->last_error = 0;
    this->kp = 0;
    this->ki = 0;

    this->param.max_out = max_out;
    if (integral_limit >= 0)
    {
        this->param.integral_higher_limit = integral_limit;
        this->param.integral_lower_limit = -integral_limit;
    }
    else
    {
        this->param.integral_higher_limit = -integral_limit;
        this->param.integral_lower_limit = integral_limit;
    }
    this->param.Ap = Ap;
    this->param.Bp = Bp;
    this->param.Cp = Cp;
    this->param.Ai = Ai;
    this->param.Ci = Ci;
    this->param.kd = kd;
}

__RAM_FUNC float variable_structure_pid::Calculate(float set, float get)
{
    this->get = get;
    this->set = set;
    this->error = this->set - this->get;
    if (fabsf(this->param.input_max_err) > 1e-6)
    {
        ABS_LIMIT(this->error, this->param.input_max_err);
    }

    this->kp = this->param.Ap + this->param.Bp * (1- exp(-this->param.Cp * ABS(this->error)));
    this->ki = this->param.Ai * exp(-this->param.Ci * ABS(this->error));

    this->pout = this->kp * this->error;
    this->iout += this->ki * this->error;
    this->dout = this->param.kd * (this->error - this->last_error);

    VAL_LIMIT(this->iout, this->param.integral_lower_limit, this->param.integral_higher_limit);
    this->out = this->pout + this->iout + this->dout;
    ABS_LIMIT(this->out, this->param.max_out);

    if (!this->enable_flag)
    {
        this->out = 0;
    }
    return (this->out);
}
