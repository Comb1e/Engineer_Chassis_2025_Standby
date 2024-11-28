//
// Created by CYK on 2024/11/27.
//

#include "PID.h"

#include "User_Lib.h"

__RAM_FUNC void pid::Init(float kp, float ki, float kd, float integral_limit, float max_out)
{
    this->param.p = kp;
    this->param.i = ki;
    this->param.d = kd;
    if(integral_limit >= 0)
    {
        this->param.integral_higher_limit = integral_limit;
        this->param.integral_lower_limit = -integral_limit;
    }
    else
    {
        this->param.integral_higher_limit = -integral_limit;
        this->param.integral_lower_limit = integral_limit;
    }
    this->param.max_out = max_out;

    this->pout = 0;
    this->iout = 0;
    this->dout = 0;
    this->error = 0;
    this->last_error = 0;
    this->penultimate_error = 0;
    this->enable_flag = true;
    this->out = 0;
}

float pid::Calculate(float set, float get)
{
    this->error = set - get;

    this->pout = this->param.p * this->error;
    this->iout += this->param.i * this->error;
    this->dout = this->param.d * (this->error - this->last_error);

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

