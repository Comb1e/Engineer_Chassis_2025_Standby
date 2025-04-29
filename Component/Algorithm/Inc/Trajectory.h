//
// Created by CYK on 2024/12/2.
//

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "Global_CFG.h"

/*-----------------机械臂步长-----------------------*/
#define ARM_TRAJECTORY_VEL_XYZ              0.3f
#define ARM_TRAJECTORY_VEL_RPY              0.1f

#define HOME_ARM_TRAJECTORY_VEL_XYZ              0.4f
#define HOME_ARM_TRAJECTORY_VEL_RPY              0.2f

enum traj_item_e : uint8_t
{
    X = 0,
    Y,
    Z,
    YAW,
    PITCH,
    ROLL,
    ARM_YAW,
    ARM_PITCH,
    TRAJ_ITEM_NUM
};

#ifdef __cplusplus
}
#endif

class Trajectory_Device
{
private:

    float final;
    float target_count;
    float already_count;
    float initial;
    float basic_step;
    float max_error;

    bool step_protected_flag;
public:
    Trajectory_Device(float max_error,float basic_step);

    float track_point;

    __RAM_FUNC void Update_Data();
    __RAM_FUNC void Change_Basic_Step(float new_step);
    __RAM_FUNC void Change_Target_Cnt_Based_On_New_Final(float new_final);
    __RAM_FUNC bool Check_Track_Point_As_Final() const;
    __RAM_FUNC void Set_Posture(float set);
    __RAM_FUNC void Add_Posture(float delta);
    void Set_Step_Protected();
    void Close_Step_Protected();

    friend class Arm_Device;
};

#endif //TRAJECTORY_H
