//
// Created by CYK on 2024/12/2.
//

#include "Drv_Arm.h"
#include <math.h>
#include <dsp/fast_math_functions.h>
#include "Drv_Chassis.h"
#include "Drv_Info.h"
#include "rotation_matrix.h"

Arm_Device arm;

Arm_Device::Arm_Device():
trajectory{Trajectory_Device(XYZ_ERROR, ARM_TRAJECTORY_VEL_XYZ),
           Trajectory_Device(XYZ_ERROR, ARM_TRAJECTORY_VEL_XYZ),
           Trajectory_Device(XYZ_ERROR, ARM_TRAJECTORY_VEL_XYZ),
           Trajectory_Device(RYP_ERROR, ARM_TRAJECTORY_VEL_RPY),
           Trajectory_Device(RYP_ERROR, ARM_TRAJECTORY_VEL_RPY),
           Trajectory_Device(RYP_ERROR, ARM_TRAJECTORY_VEL_RPY),
           Trajectory_Device(RYP_ERROR, ARM_TRAJECTORY_VEL_RPY),
           Trajectory_Device(RYP_ERROR,ARM_TRAJECTORY_VEL_RPY)}
{
    this->enable_flag = false;
    this->arm_chassis_cooperate_flag = false;
    this->init_cnt = 0;
}

void Arm_Device::Init(CAN_HandleTypeDef *hcan, uint32_t rx_stdid, uint32_t tx_stdid, osSemaphoreId_t rx_sem)
{
    this->can_device.RX_Add(hcan,rx_stdid,Arm_RX_Data_Update_Callback,rx_sem);
    this->can_device.TX_Init(hcan,tx_stdid,(uint8_t *)&this->can_tx_data,0x08);

    this->Limit_Init();
    this->Posture_Init();
}

void Arm_RX_Data_Update_Callback(can_device_t *can_device, uint8_t *rx_data)
{
    Arm_Device *arm = Container_Of(can_device, Arm_Device, can_device);

    auto *fb_data = (arm_rec_data_t *) rx_data;

    arm->ptz_reset_ok_flag = true;

    arm->fb_current_data.x = (float) fb_data->x - ARM_X_OFFSET;
    arm->fb_current_data.y = (float) fb_data->y - ARM_Y_OFFSET;
    arm->fb_current_data.z = (float) fb_data->z - ARM_Z_OFFSET;

    arm->fb_current_data.sucker_pitch_deg = (float) fb_data->sucker_pitch;
    arm->fb_current_data.sucker_roll_deg = (float) fb_data->sucker_roll;
    arm->fb_current_data.sucker_yaw_deg = (float) fb_data->sucker_yaw;

    arm->fb_current_data.arm_yaw = info.rx_data.fb_arm_yaw;
    arm->fb_current_data.arm_pitch = info.rx_data.fb_arm_pitch;
}

void Arm_Device::Limit_Init()
{
    this->min_limit[X] = X_MIN;
    this->max_limit[X] = X_TOTAL_MAX;
    this->min_limit[Y] = Y_MIN;
    this->max_limit[Y] = Y_MAX;
    this->min_limit[Z] = Z_MIN;
    this->max_limit[Z] = Z_MAX;
    this->min_limit[ARM_YAW] = ARM_YAW_MIN;
    this->max_limit[ARM_YAW] = ARM_YAW_MAX;
    this->min_limit[ARM_PITCH] = ARM_PITCH_MIN;
    this->max_limit[ARM_PITCH] = ARM_PITCH_MAX;
    this->min_limit[ROLL] = -ROLL_LIMIT;
    this->max_limit[ROLL] = ROLL_LIMIT;
    this->min_limit[YAW] = -YAW_LIMIT;
    this->max_limit[YAW] = YAW_LIMIT;
    this->max_limit[PITCH] = PITCH_LIMIT;
    this->min_limit[PITCH] = -PITCH_LIMIT;
}

void Arm_Device::Check_Lost()
{
    osStatus_t status = osSemaphoreAcquire(ArmUpdateBinarySemHandle,15);
    if(status == osOK)
    {
        this->connect_flag = true;
    }
    else
    {
        this->connect_flag = false;
    }
}

bool Arm_Device::Check_Enable() const
{
    return this->enable_flag;
}

void Arm_Device::Update_Enable()
{
    if(rc.ctrl_protection.connect_flag)
    {
        this->enable_flag = true;
    }
    else
    {
        this->enable_flag = false;
    }
}

void Arm_Device::Update_Control()
{
    this->Update_Limit_Basic_Data();
    this->Update_Limit();

    this->Update_Final();
    this->Update_Trajectory_Data();
    this->Update_Ctrl_Data();
    this->CAN_Set();
}

void Arm_Device::CAN_Send_MSG()
{
    taskENTER_CRITICAL();
    this->can_device.Send_MSG();
    taskEXIT_CRITICAL();
}

void Arm_Device::CAN_Set()
{
    taskENTER_CRITICAL();

    this->can_tx_data.x = (uint16_t)roundf((this->ctrl_data.x + ARM_X_OFFSET) *2.f);
    this->can_tx_data.y = (uint16_t)roundf((this->ctrl_data.y + ARM_Y_OFFSET)*2.f);
    this->can_tx_data.z = (uint16_t)roundf((this->ctrl_data.z + ARM_Z_OFFSET)*2.f);

    this->can_tx_data.sucker_roll = (int16_t)roundf((this->ctrl_data.sucker_roll_deg) * 2.f);
    this->can_tx_data.sucker_pitch = (int16_t)roundf((this->ctrl_data.sucker_pitch_deg) * 2.f);
    this->can_tx_data.sucker_yaw = (int16_t)roundf((this->ctrl_data.sucker_yaw_deg) * 2.f);

    info.tx_raw_data.arm_pitch_deg = this->ctrl_data.arm_pitch;
    info.tx_raw_data.arm_yaw_deg = this->ctrl_data.arm_yaw;

    taskEXIT_CRITICAL();
}

void Arm_Device::Set_Point_Final_Posture(traj_item_e point, float posture)
{
    this->trajectory_final[point] = posture;
}

void Arm_Device::Set_Point_Posture(traj_item_e point, float posture)
{
    VAL_LIMIT(posture, this->min_limit[point], this->max_limit[point]);
    this->Set_Point_Final_Posture(point, posture);
    this->trajectory[point].Set_Posture(posture);
}

void Arm_Device::Posture_Init()
{
    this->Set_Point_Posture(X, INIT_ARM_X);
    this->Set_Point_Posture(Y, INIT_ARM_Y);
    this->Set_Point_Posture(Z, INIT_ARM_Z);
    this->Set_Point_Posture(YAW, INIT_SUCKER_YAW);
    this->Set_Point_Posture(PITCH, INIT_SUCKER_PITCH);
    this->Set_Point_Posture(ROLL, INIT_SUCKER_ROLL);
    this->Set_Point_Posture(ARM_YAW, INIT_ARM_YAW_DEG);
    this->Set_Point_Posture(ARM_PITCH,INIT_ARM_PITCH_DEG);
}

void Arm_Device::Update_Limit_Basic_Data()
{
    this->limit_basic_data.arm_xoy_length = ARM_LENGTH - ARM_LENGTH_2_ACT + ARM_LENGTH_2_ACT * arm_cos_f32(this->trajectory[ARM_PITCH].track_point * PI / 180.0f);
    this->limit_basic_data.arm_yaw_radian = this->trajectory[ARM_YAW].track_point * PI / 180.0f;
    this->limit_basic_data.arm_x_length = this->limit_basic_data.arm_xoy_length * arm_cos_f32(this->limit_basic_data.arm_yaw_radian);
    this->limit_basic_data.arm_y_length = this->limit_basic_data.arm_xoy_length * arm_sin_f32(this->limit_basic_data.arm_yaw_radian);
    this->limit_basic_data.y_base = this->trajectory[Y].track_point - this->limit_basic_data.arm_y_length;
}

void Arm_Device::Update_Limit()
{
    this->Update_X_Limit();
    this->Update_Y_Limit();
    this->Update_Z_Limit();
    this->Update_Pitch_Limit();
}

void Arm_Device::Update_X_Limit()
{
    min_limit[X] = this->limit_basic_data.arm_x_length;
    max_limit[X] = FRAME_EXTENSION_MAX + min_limit[X];
}

void Arm_Device::Update_Y_Limit()
{
    min_limit[Y] = this->limit_basic_data.arm_y_length;
    max_limit[Y] = FRAME_SLIDING_MAX + min_limit[Y];
}

void Arm_Device::Update_Z_Limit()
{
    min_limit[Z] = FRAME_UPLIFT_MIN - ARM_LENGTH_2_ACT * arm_sin_f32(this->trajectory[ARM_PITCH].track_point * PI / 180.0f);
    max_limit[Z] = min_limit[Z] + FRAME_UPLIFT_MAX;
}

void Arm_Device::Update_Pitch_Limit()
{
    min_limit[PITCH] = this->trajectory[ARM_PITCH].track_point - 90.0f;
    max_limit[PITCH] = this->trajectory[ARM_PITCH].track_point + 90.0f;
}

void Arm_Device::Update_Final()
{
    for (traj_item_e point = X; point < TRAJ_ITEM_NUM; point = (traj_item_e) (point + 1))
    {
        if(chassis.set_vel.x == 0 && chassis.set_vel.y == 0 && chassis.set_vel.spin == 0)
        {
            if(point == X)
            {
                 if(this->trajectory_final[X] > max_limit[X])
                {
                     this->arm_chassis_cooperate_flag = true;
                     this->chassis_move_data.x += this->trajectory_final[X] - max_limit[X];
                     this->trajectory[X].Change_Target_Cnt_Based_On_New_Final(max_limit[X]);
                     this->trajectory_final[X] = max_limit[X];
                     this->trajectory[X].final = max_limit[X];

                     chassis.arm_need_cnt = 0;
                }
                else if(this->trajectory_final[X] < min_limit[X] && chassis.set_vel.x == 0 && chassis.set_vel.y == 0 && chassis.set_vel.spin == 0)
                {
                     this->arm_chassis_cooperate_flag = true;
                     this->chassis_move_data.x += this->trajectory_final[X] - min_limit[X];
                     this->trajectory[X].Change_Target_Cnt_Based_On_New_Final(min_limit[X]);
                     this->trajectory_final[X] = min_limit[X];
                     this->trajectory[X].final = min_limit[X];

                     chassis.arm_need_cnt = 0;
                }
                else
                {
                    VAL_LIMIT(this->trajectory_final[X],this->min_limit[X],this->max_limit[X]);
                    this->trajectory[X].Change_Target_Cnt_Based_On_New_Final(this->trajectory_final[X]);
                    this->trajectory[X].final = this->trajectory_final[X];
                }
            }
            else if(point == Y)
            {
                if(this->trajectory_final[Y] > max_limit[Y] && chassis.set_vel.x == 0 && chassis.set_vel.y == 0 && chassis.set_vel.spin == 0)
                {
                    this->arm_chassis_cooperate_flag = true;
                    this->chassis_move_data.y += this->trajectory_final[Y] - max_limit[Y];
                    this->trajectory[Y].Change_Target_Cnt_Based_On_New_Final(max_limit[Y]);
                    this->trajectory_final[Y] = max_limit[Y];
                    this->trajectory[Y].final = max_limit[Y];

                    chassis.arm_need_cnt = 0;
                }
                else if(this->trajectory_final[Y] < min_limit[Y] && chassis.set_vel.x == 0 && chassis.set_vel.y == 0 && chassis.set_vel.spin == 0)
                {
                    this->arm_chassis_cooperate_flag = true;
                    this->chassis_move_data.y += this->trajectory_final[Y] - min_limit[Y];
                    this->trajectory[Y].Change_Target_Cnt_Based_On_New_Final(min_limit[Y]);
                    this->trajectory_final[Y] = min_limit[Y];
                    this->trajectory[Y].final = min_limit[Y];

                    chassis.arm_need_cnt = 0;
                }
                else
                {
                    VAL_LIMIT(this->trajectory_final[Y],this->min_limit[Y],this->max_limit[Y]);
                    this->trajectory[Y].Change_Target_Cnt_Based_On_New_Final(this->trajectory_final[Y]);
                    this->trajectory[Y].final = this->trajectory_final[Y];
                }
            }
            else
            {
                VAL_LIMIT(this->trajectory_final[point],min_limit[point],max_limit[point]);
                this->trajectory[point].Change_Target_Cnt_Based_On_New_Final(this->trajectory_final[point]);
                this->trajectory[point].final = this->trajectory_final[point];
            }
        }
        else
        {
            VAL_LIMIT(this->trajectory_final[point],min_limit[point],max_limit[point]);
            this->trajectory[point].Change_Target_Cnt_Based_On_New_Final(this->trajectory_final[point]);
            this->trajectory[point].final = this->trajectory_final[point];

            chassis.Reset_Total_Rounds();
        }
    }
}

void Arm_Device::Update_Trajectory_Data()
{
    for (traj_item_e point = X; point < TRAJ_ITEM_NUM; point = (traj_item_e) (point + 1))
    {
        this->trajectory[point].Update_Data();
    }
}

void Arm_Device::Update_Ctrl_Data()
{
    this->Set_X(this->trajectory[X].track_point);
    this->Set_Y(this->trajectory[Y].track_point);
    this->Set_Z(this->trajectory[Z].track_point);
    this->Set_Pitch(this->trajectory[PITCH].track_point);
    this->Set_Roll(this->trajectory[ROLL].track_point);
    this->Set_Yaw(this->trajectory[YAW].track_point);
    this->Set_Arm_Pitch(this->trajectory[ARM_PITCH].track_point);
    this->Set_Arm_Yaw(this->trajectory[ARM_YAW].track_point);
}

void Arm_Device::Set_X(float track_point)
{
    this->ctrl_data.x = track_point;
    VAL_LIMIT(this->ctrl_data.x,min_limit[X],max_limit[X]);
}

void Arm_Device::Set_Y(float track_point)
{
    this->ctrl_data.y = track_point;
    VAL_LIMIT(this->ctrl_data.y,min_limit[Y],max_limit[Y]);
}

void Arm_Device::Set_Z(float track_point)
{
    this->ctrl_data.z = track_point;
    VAL_LIMIT(this->ctrl_data.z,min_limit[Z],max_limit[Z]);
}

void Arm_Device::Set_Pitch(float track_point)
{
    this->ctrl_data.sucker_pitch_deg = track_point;
    VAL_LIMIT(this->ctrl_data.sucker_pitch_deg,min_limit[PITCH],max_limit[PITCH]);
}

void Arm_Device::Set_Roll(float track_point)
{
    this->ctrl_data.sucker_roll_deg = track_point;
    VAL_LIMIT(this->ctrl_data.sucker_roll_deg,min_limit[ROLL],max_limit[ROLL]);
}

void Arm_Device::Set_Yaw(float track_point)
{
    this->ctrl_data.sucker_yaw_deg = track_point;
    VAL_LIMIT(this->ctrl_data.sucker_roll_deg,min_limit[YAW],max_limit[YAW]);
}

void Arm_Device::Set_Arm_Pitch(float track_point)
{
    this->ctrl_data.arm_pitch = track_point;
    VAL_LIMIT(this->ctrl_data.arm_pitch,min_limit[ARM_PITCH],max_limit[ARM_PITCH]);
}

void Arm_Device::Set_Arm_Yaw(float track_point)
{
    this->ctrl_data.arm_yaw = track_point;
    VAL_LIMIT(this->ctrl_data.arm_yaw,min_limit[ARM_YAW],max_limit[ARM_YAW]);
}

void Arm_Device::Add_Point_Target_Pos_From_Control(traj_item_e point, float delta)
{
    float step;
    if (fabsf(delta - 0.0f) > 0.0001f)
    {
        if(point<=Z)
        {
            step = ARM_TRAJECTORY_VEL_XYZ / KB_CONTROL_CYCLE * ARM_CONTROL_CYCLE;
        }
        else
        {
            step = ARM_TRAJECTORY_VEL_RPY / KB_CONTROL_CYCLE * ARM_CONTROL_CYCLE;
        }

        if(fabsf(delta) > 1.0f)
        {
            step *= fabsf(delta);
        }
        this->trajectory[point].Change_Basic_Step(step);
    }

    if(fabsf(delta) <= 1.0f)
    {
        this->trajectory_final[point] += delta * this->trajectory[point].basic_step;
    }
    else if(fabsf(delta) > 1.0f)
    {
        this->trajectory_final[point] += delta/ fabsf(delta) *this->trajectory[point].basic_step;
    }

    if(point !=X && point !=Y)
    {
        VAL_LIMIT(this->trajectory_final[point],min_limit[point],max_limit[point]);
    }
}

void Arm_Device::Clean_Control()
{
    this->Set_FeedBack_As_Target();
    this->arm_chassis_cooperate_flag = false;
    this->chassis_move_data.x = 0;
    this->chassis_move_data.y = 0;
}

void Arm_Device::Set_FeedBack_As_Target()
{
    this->Set_Point_Posture(X,this->fb_current_data.x);
    this->Set_Point_Posture(Y,this->fb_current_data.y);
    this->Set_Point_Posture(Z,this->fb_current_data.z);
    this->Set_Point_Posture(ARM_YAW,this->fb_current_data.arm_yaw);
    this->Set_Point_Posture(ARM_PITCH,this->fb_current_data.arm_pitch);
    this->Set_Point_Posture(YAW,this->fb_current_data.sucker_yaw_deg);
    this->Set_Point_Posture(PITCH,this->fb_current_data.sucker_pitch_deg);
    this->Set_Point_Posture(ROLL,this->fb_current_data.sucker_roll_deg);
}

bool Arm_Device::Check_Init_Completely()
{
    if(this->init_cnt < 5000)
    {
        this->init_cnt++;
        return false;
    }
    return true;
}

void Arm_Device::Add_Point_Target_Pos(traj_item_e point, float delta_target)
{
    this->trajectory_final[point] += delta_target;
    if (point != X && point != Y)
    {
        VAL_LIMIT(this->trajectory_final[point], this->min_limit[point], this->max_limit[point]);
    }
}

/**
 * @brief 沿吸盘的三个方向推
 * @param point
 * @param compensation
 * @param distance
 * @param vel
 */
void Arm_Device::Rectilinear_Motion(traj_item_e point,float compensation, float distance,float vel)
{
    if(point >= YAW)
    {
        return;
    }

    float mat[9] = {0};
    float dx, dy, dz;
    float dx_v, dy_v, dz_v;


    set_rotMatrix_from_euler_zyx((this->trajectory[YAW].track_point / 180.0f) * PI,
                                 ((this->trajectory[PITCH].track_point + compensation) / 180.0f) * PI,
                                 (this->trajectory[ROLL].track_point / 180.0f) * PI, mat);
    dx = distance * mat[point];
    dy = distance * mat[point + 3];
    dz = distance * mat[point + 6];

    dx_v = vel * mat[point];
    dy_v = vel * mat[point + 3];
    dz_v = vel * mat[point + 6];

    this->Add_Point_Target_Pos(X, dx);
    this->Add_Point_Target_Pos(Y, dy);
    this->Add_Point_Target_Pos(Z, dz);

    if (fabsf(dx_v) > 0.001)
    {
        this->trajectory[X].Change_Basic_Step(dx_v);
    }
    if (fabsf(dy_v) > 0.001)
    {
        this->trajectory[Y].Change_Basic_Step(dy_v);
    }
    if (fabsf(dz_v) > 0.001)
    {
        this->trajectory[Z].Change_Basic_Step(dz_v);
    }
}

void Arm_Device::Arm_Yaw_Dir_Move(float distance, float vel)
{
    float mat[9] = {0};
    float dx, dy, dz;
    float dx_v, dy_v, dz_v;
    set_rotMatrix_from_euler_zyx((this->trajectory[ARM_YAW].track_point / 180.0f) * PI,
                                 0,
                                 0, mat);
    dx = distance * mat[0];
    dy = distance * mat[3];
    dz = distance * mat[6];

    dx_v = vel * mat[0];
    dy_v = vel * mat[3];
    dz_v = vel * mat[6];

    this->Add_Point_Target_Pos(X, dx);
    this->Add_Point_Target_Pos(Y, dy);
    this->Add_Point_Target_Pos(Z, dz);

    if (fabsf(dx_v) > 0.001)
    {
        this->trajectory[X].Change_Basic_Step(dx_v);
    }
    if (fabsf(dy_v) > 0.001)
    {
        this->trajectory[Y].Change_Basic_Step(dy_v);
    }
    if (fabsf(dz_v) > 0.001)
    {
        this->trajectory[Z].Change_Basic_Step(dz_v);
    }
}

void Arm_Device::Set_Step_Protected()
{
    for (traj_item_e point = X; point < TRAJ_ITEM_NUM; point = (traj_item_e) (point + 1))
    {
        this->trajectory[point].Set_Step_Protected();
    }
}

void Arm_Device::Close_Step_protected()
{
    for (traj_item_e point = X; point < TRAJ_ITEM_NUM; point = (traj_item_e) (point + 1))
    {
        this->trajectory[point].Close_Step_Protected();
    }
}

bool Arm_Device::Check_All_Get_To_Final()
{
    for (traj_item_e point = X; point < TRAJ_ITEM_NUM; point = (traj_item_e) (point + 1))
    {
        if(!this->trajectory[point].Check_Track_Point_As_Final())
        {
            return false;
        }
    }
    return true;
}

void Arm_Device::Set_Point_Target_Pos_Vel(traj_item_e point, float pos, float vel)
{
    this->Set_Point_Final_Posture(point,pos);
    this->trajectory[point].Change_Basic_Step(vel);
}
