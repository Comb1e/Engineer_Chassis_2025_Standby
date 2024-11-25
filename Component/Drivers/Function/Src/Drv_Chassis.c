//
// Created by CYK on 2024/11/20.
//

#include <math.h>

#include "Drv_chassis.h"
#include "BSP_Can.h"
#include "can.h"
#include "Drv_HI229UM.h"
#include "Drv_IMU.h"
#include "Mecanum.h"
#include "RTOS.h"
#include "User_Lib.h"
#include "Global_CFG.h"

chassis_t chassis;
DJI_motor_t M2006;
void Chassis_Init(chassis_t *chassis)
{
    DJI_Motor_Init(&chassis->M3508[CHASSIS_LF_NUM],false,0x201 + CHASSIS_LF_NUM,0,0,DJI_M3508,&CHASSIS_HCAN,ChassisLFUpdateBinarySemHandle,false);
    DJI_Motor_Init(&chassis->M3508[CHASSIS_LB_NUM],false,0x201 + CHASSIS_LB_NUM,0,0,DJI_M3508,&CHASSIS_HCAN,ChassisLBUpdateBinarySemHandle,false);
    DJI_Motor_Init(&chassis->M3508[CHASSIS_RB_NUM],true,0x201 + CHASSIS_RB_NUM,0,0,DJI_M3508,&CHASSIS_HCAN,ChassisRBUpdateBinarySemHandle,false);
    DJI_Motor_Init(&chassis->M3508[CHASSIS_RF_NUM],true,0x201 + CHASSIS_RF_NUM,0,0,DJI_M3508,&CHASSIS_HCAN,ChassisRFUpdateBinarySemHandle,false);
    DJI_Motor_Init(&M2006,false,0x205,0,0,DJI_M2006,&CHASSIS_HCAN,NULL,false);


    CAN1_Filter_ID_Init(chassis->M3508[CHASSIS_LF_NUM].can_device.basic.rx);
    CAN1_Filter_ID_Init(chassis->M3508[CHASSIS_LB_NUM].can_device.basic.rx);
    CAN1_Filter_ID_Init(chassis->M3508[CHASSIS_RB_NUM].can_device.basic.rx);
    CAN1_Filter_ID_Init(chassis->M3508[CHASSIS_RF_NUM].can_device.basic.rx);
    //
    CAN1_Filter_ID_Init(M2006.can_device.basic.rx);
    //

    PID_Init(&chassis->M3508[CHASSIS_LF_NUM].pid_loc,1,0,0,100,1);
    PID_Init(&chassis->M3508[CHASSIS_LB_NUM].pid_loc,1,0,0,100,1);
    PID_Init(&chassis->M3508[CHASSIS_RB_NUM].pid_loc,1,0,0,100,1);
    PID_Init(&chassis->M3508[CHASSIS_RF_NUM].pid_loc,1,0,0,100,1);

    PID_Init(&chassis->M3508[CHASSIS_LF_NUM].pid_vel,1,0,0,100,DJI_MOTOR_MAX_CURRENT_M3508);
    PID_Init(&chassis->M3508[CHASSIS_LB_NUM].pid_vel,1,0,0,100,DJI_MOTOR_MAX_CURRENT_M3508);
    PID_Init(&chassis->M3508[CHASSIS_RB_NUM].pid_vel,1,0,0,100,DJI_MOTOR_MAX_CURRENT_M3508);
    PID_Init(&chassis->M3508[CHASSIS_RF_NUM].pid_vel,1,0,0,100,DJI_MOTOR_MAX_CURRENT_M3508);

    //
    PID_Init(&M2006.pid_loc,1,0,0,100,1);
    PID_Init(&M2006.pid_vel,1000,0,0,100,1);
    PID_Init(&M2006.pid_tor,10,0,0,100,1);
    //

    chassis->state.enable_flag = true;
    chassis->state.lost_flag = true;
    chassis->state.ready_flag = false;
    chassis->state.arm_need_flag = false;
    chassis->state.super_rotate_flag = false;
    chassis->state.robot_set_easter_use_flag = false;

    chassis->control_type = Speed;

    chassis->direction_angle = 0.0f;

    chassis->velocity.speedY = 0;
    chassis->velocity.speedX = 0;
    chassis->velocity.speedSpin = 0;
    chassis->velocity.rc_set_spin = 0;
    chassis->velocity.small_gyroscope_speed = CHASSIS_SMALL_GYROSCOPE_SPEED;
    chassis->velocity.vel_max.kb = CHASSIS_VEL_KB_MAX;
    chassis->velocity.vel_max.rc = CHASSIS_VEL_RC_MAX;
    chassis->velocity.vel_max.total = CHASSIS_VEL_TOTAL_MAX;
}

void Chassis_PowerCtrl_Data_Init(chassis_t *chassis)
{
    chassis->power_control_data.para_H_init = 6.7;
    for(uint8_t i = 0;i<4;i++)
    {
        chassis->power_control_data.para_H[i] = chassis->power_control_data.para_H_init;
        chassis->power_control_data.para_R[i] = 0.01f;//0.0005f;
    }

    chassis->power_control_data.para_Hinc = 0.0008f;
    chassis->power_control_data.para_Hkp = 0.0002f;//0.002f;
    chassis->power_control_data.para_H_sum_i = 0.0f;
    chassis->power_control_data.limit_power = 0.0f;
    chassis->power_control_data.now_power = 0.0f;
    chassis->power_control_data.limit_power =CHASSIS_POWER_LIMIT;
}

void Chassis_PowerCtrl_Update(chassis_t *chassis)
{
    //光解算出公式中的k
    float res_k,temp_a,temp_b , qa,qb,qc;
    qc= chassis->power_control_data.limit_power;
    for(uint8_t i = 0;i<4;i++){
        temp_a = chassis->power_control_data.para_R[i] * chassis->M3508[i].pid_vel.out * chassis->M3508[i].pid_vel.out;
        qa += temp_a;
        temp_b = chassis->power_control_data.para_H[i] * fabsf(chassis->M3508[i].pid_vel.out) * fabsf(chassis->M3508[i].current_data.speed_rpm);
        qb += temp_b;
    }

    if(qa <= 0){
        qa = 0.000001f;
    }

    chassis->power_control_data.qA = qa;
    chassis->power_control_data.qB = qb;
    chassis->power_control_data.qC = qc;

    if(fabsf(qa) < 0.000000000000000001f)
    {
        res_k = 1.0f;
    }
    else
    {
        res_k = (float)(-qb + sqrtf(qb * qb + 4.0f * qa * qc / 80.0f)) / (2.0f * qa);
    }

    if(res_k > 1)
    {
        res_k = 1.0f;
    }
    else
    {
        if(fabsf(qa) > 0.000000000001f && fabsf(qb) >0.00000000000001f)
        {//todo 改成pid
            float power_error = chassis->power_control_data.now_power - chassis->power_control_data.limit_power;
            chassis->power_control_data.para_H_sum_i += chassis->power_control_data.para_Hinc * power_error;
            VAL_LIMIT(chassis->power_control_data.para_H_sum_i,-0.5f,7.0f);

            for(uint8_t i=0;i<4;i++)
            {
                chassis->power_control_data.para_H[i] = chassis->power_control_data.para_H[i] + chassis->power_control_data.para_Hkp * power_error;
                VAL_LIMIT(chassis->power_control_data.para_H[i],3.0f,22.0f);
            }
            //todo 裁判系统丢失保护
        }
    }
    VAL_LIMIT(res_k,0.0f,1.0f);
    chassis->power_control_data.k = res_k;
}

void Set_PowerCtrl_now_power(chassis_t *chassis,float now_power)
{
    chassis->power_control_data.now_power = now_power;
}

bool Chassis_Check_Init_Completely(const chassis_t *chassis)
{
    if(chassis->M3508[0].state.zero_offset_flag && chassis->M3508[1].state.zero_offset_flag && chassis->M3508[2].state.zero_offset_flag && chassis->M3508[3].state.zero_offset_flag)
    {
        return true;
    }
    return false;
}

void Chassis_Tof_Init(chassis_t *chassis,CAN_HandleTypeDef *hcan, uint32_t rx_id, osSemaphoreId_t rx_sem)
{
    chassis->tof.can_device.basic.hcan = hcan;
    chassis->tof.can_device.basic.rx_sem = rx_sem;
    chassis->tof.can_device.basic.rx.rx_id = rx_id;
    chassis->tof.can_device.basic.rx.rx_callback = TOF_RX_Data_Update_CallBack;

    PID_Init(&chassis->tof.dist_pid,0.0001f,0.0f,0.0005f,0.01f,0.1f);
    PID_Init(&chassis->tof.align_pid,0.08f,0.0f,0.01f,0.8f,0.8f);
}

void TOF_RX_Data_Update_CallBack(uint32_t std_id,const uint8_t *rx_data)
{
    chassis.tof.align_data.left_dist = rx_data[0] << 8 | rx_data[1];
    chassis.tof.align_data.right_dist = rx_data[2] << 8 | rx_data[3];
}

void Chassis_Update_Ready(chassis_t *chassis)
{
    for(int i=0;i<4;i++)
    {
        DJI_Motor_Update_Ready(&chassis->M3508[i]);
    }

    if(chassis->M3508[0].state.ready_flag && chassis->M3508[1].state.ready_flag && chassis->M3508[2].state.ready_flag && chassis->M3508[3].state.ready_flag)
    {
        chassis->state.ready_flag = true;
        return;
    }

    chassis->state.ready_flag = false;
    if(chassis->M3508[0].state.lost_flag || chassis->M3508[1].state.lost_flag || chassis->M3508[2].state.lost_flag || chassis->M3508[3].state.lost_flag)
    {
        chassis->state.lost_flag = true;
        return;
    }
    chassis->state.lost_flag = false;
}

void Chassis_Set_Free(chassis_t *chassis)
{
    for(int i=0;i<4;i++)
    {
        DJI_Motor_Set_Free(&chassis->M3508[i]);
    }
}

__RAM_FUNC void Chassis_Update_Position_Ctrl(chassis_t *chassis)
{
    if(hi229um.state.zero_offset_flag)
    {
        PID_Error_Calculate_N_Loc(&chassis->pos_rot_pid,chassis->pos_yaw_angle,HI229UM_Get_Yaw_Total_Deg());
        Chassis_Add_Position_Spin(chassis,2 * PI * PID_Calculate(&chassis->pos_rot_pid));
        Chassis_Motor_SolverSet(chassis->M3508,chassis->position.positionX,chassis->position.positionY,chassis->position.positionSpin);
    }
}

void Chassis_Add_Position_Spin(chassis_t *chassis,float delta_spin)
{
    chassis->position.positionSpin += delta_spin;
    VAL_LIMIT(chassis->position.positionSpin,-30,30);
}

__RAM_FUNC void Chassis_Update_Speed_Ctrl(chassis_t *chassis)
{
    float speedX = 0.0f, speedY = 0.0f, speedSpin = 0.0f;
    static bool use_hi229um_flag;

    Get_Slope_Speed(&chassis->kb_x_speed);
    Get_Slope_Speed(&chassis->kb_y_speed);
    Chassis_Update_Align(chassis);

    Chassis_Set_Vel_X(chassis,chassis->kb_x_speed.out);
    Chassis_Set_Vel_Y(chassis,chassis->kb_y_speed.out);

    speedX = chassis->velocity.speedX * arm_cos_f32(chassis->direction_angle * 2 * PI)
        + chassis->velocity.speedY * arm_sin_f32(chassis->direction_angle * 2 * PI) + chassis->tof.align_data.vel_x;
    speedY = -chassis->velocity.speedX * arm_sin_f32(chassis->direction_angle * 2 * PI)
        + chassis->velocity.speedY * arm_cos_f32(chassis->direction_angle * 2 * PI);


    if (chassis->state.small_gyroscope_flag)
    {
        speedSpin = chassis->velocity.small_gyroscope_speed;
    }
    else
    {
        if (!hi229um.state.enable_flag || hi229um.state.lost_flag)
        {
            Chassis_Close_Yaw_Spin(chassis);
            Chassis_Set_Vel_Spin(chassis,10 * chassis->velocity.rc_set_spin + 30.F*chassis->tof.align_data.vel_spin + 0.5f * chassis->tof.align_data.delta_rounds);
            speedSpin = chassis->velocity.speedSpin ;
        }
        else
        {
            use_hi229um_flag = true;
            if (hi229um.state.zero_offset_flag)
            {
                chassis->yaw_round_set += (0.08f * chassis->velocity.rc_set_spin + 0.05f*chassis->tof.align_data.vel_spin) * chassis->yaw_round_set_proportion;
#if MAHONY
                PID_Error_Calculate_N_Loc(&chassis->rot_pid,chassis->yaw_round_set,HI229UM_Get_Yaw_Total_Deg());
                Chassis_Set_Vel_Spin(chassis,PID_Calculate(&chassis->rot_pid));
#else
                chassis->set_speed_spin(chassis->rotpid.calculate(chassis->yaw_round_set,  chassis->hi229um->get_yaw_total_deg()));
#endif
//              speedSpin = 0.2f*chassis->velocity.speedSpin + 0.8f* speedSpin;//尝试滤波
                speedSpin = chassis->velocity.speedSpin ;
            }
            else
            {
                speedSpin = 0.0f;
                chassis->yaw_round_set = 0;
            }
        }
    }

    if (chassis->state.super_rotate_flag)
    {
        Super_Chassis_Motor_SolverSet(chassis->M3508, speedX, speedY, speedSpin);
    }
    else
    {
        Chassis_Motor_SolverSet(chassis->M3508, speedX, speedY, speedSpin);
    }
    Chassis_PowerCtrl_Update(chassis);
}

void Chassis_Update_Align(chassis_t *chassis)
{
    if(!chassis->state.tof_lost_flag)
    {
        chassis->tof.align_data.beta = atanf((chassis->tof.align_data.right_dist - chassis->tof.align_data.left_dist) / TOF_DEVICE_DISTANCE);
        chassis->tof.align_data.center_dist = (chassis->tof.align_data.right_dist+ chassis->tof.align_data.left_dist) / 2.f;
    }

    if(chassis->state.enable_align_flag && !chassis->state.tof_lost_flag)
    {

        if(ABS(chassis->tof.align_data.beta) <= ALIGN_CRITICAL_ANGLE && chassis->tof.align_data.center_dist <= (chassis->tof.align_data.target_dist + ALIGN_DELTA_DISTANCE))
        {
            chassis->tof.align_data.delta_rounds = chassis->tof.align_data.beta/(2*PI);
            PID_Error_Calculate_N_Loc(&chassis->tof.dist_pid,chassis->tof.align_data.target_dist, chassis->tof.align_data.center_dist);
            chassis->tof.align_data.vel_x = -PID_Calculate(&chassis->tof.dist_pid);
            if(chassis->tof.align_data.left_dist < ALIGN_CRITICAL_DISTANCE || chassis->tof.align_data.right_dist < ALIGN_CRITICAL_DISTANCE)
            {
                PID_Error_Calculate_N_Loc(&chassis->tof.align_pid,0,chassis->tof.align_data.delta_rounds);
                chassis->tof.align_data.vel_spin = -PID_Calculate(&chassis->tof.align_pid);
            }else
            {
                chassis->tof.align_data.vel_spin = 0;
            }
        }
        else
        {
            chassis->tof.align_data.vel_x = 0.0f;
            chassis->tof.align_data.vel_y = 0.0f;
            chassis->tof.align_data.vel_spin = 0.0f;
            chassis->tof.align_data.delta_rounds = 0.0f;
        }
    }
    else
    {
        chassis->tof.align_data.vel_x = 0.0f;
        chassis->tof.align_data.vel_y = 0.0f;
        chassis->tof.align_data.vel_spin = 0.0f;
        chassis->tof.align_data.beta = 0.0f;
        chassis->tof.align_data.delta_rounds = 0.0f;
    }
}

void Chassis_Set_Vel_X(chassis_t *chassis,float vel_x)
{
    vel_x = ABS_Limit(vel_x,chassis->velocity.vel_max.total);
    chassis->velocity.speedX = vel_x;
}

void Chassis_Set_Vel_Y(chassis_t *chassis,float vel_y)
{
    vel_y = ABS_Limit(vel_y,chassis->velocity.vel_max.total);
    chassis->velocity.speedY = vel_y;
}

void Chassis_Set_Vel_Spin(chassis_t *chassis,float vel_spin)
{
    vel_spin = ABS_Limit(vel_spin,chassis->velocity.vel_max.total * 0.8);
    chassis->velocity.speedSpin = vel_spin;
}

void Chassis_Close_Yaw_Spin(chassis_t *chassis)
{
    IMU_Set_Current_As_Offset();
    HI229UM_Set_Current_As_Offset();

    chassis->yaw_round_set = 0.0f;
    chassis->velocity.speedSpin = 0.0f;
}