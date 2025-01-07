//
// Created by CYK on 2024/11/28.
//

#ifndef MECANUM_H
#define MECANUM_H

#include "Drv_DJI_Motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

/* the radius of wheel(mm) */
#ifndef RADIUS
#define RADIUS 77
#endif
/* the perimeter of wheel(mm) */  //车轮周长 2*PI*R
#define PERIMETER 483.8f

/* g_chassis motor use 3508 */
/* the deceleration ratio of g_chassis motor */
#define MOTOR_DECELE_RATIO (1.0f / 19.0f)

#define WHEELPERIMETER 395.0f

#define WHEELPERIMETER_X  395.f
#define WHEELPERIMETER_Y  373.f

/* wheel track distance(mm) */ //轨距 左右轮之间的距离
#define WHEELTRACK (450.49f)
/* wheelbase distance(mm) */ //轮距 前后轮之车轮轴距离
#define WHEELBASE (362.9f)

/* g_gimbal is relative to g_chassis center x axis offset(mm) */
#define ROTATE_X_OFFSET (-165.0f)
/* g_gimbal is relative to g_chassis center y axis offset(mm) */
#define ROTATE_Y_OFFSET (0.0f)

#define ECD_RATIO  (WHEELPERIMETER * MOTOR_DECELE_RATIO)

#define ECD_RATIO_X  (WHEELPERIMETER_X * MOTOR_DECELE_RATIO)
#define ECD_RATIO_Y  (WHEELPERIMETER_Y * MOTOR_DECELE_RATIO)

#ifndef RADIAN_COEF
#define RADIAN_COEF 57.3f //弧度到角度
#endif

#ifdef __cplusplus
}
#endif

void Chassis_Motor_Solver_Set(DJI_Motor_Device wheel[],float vel_x,float vel_y,float vel_spin,float vel_max);
__RAM_FUNC void Chassis_Motor_Loc_SolverSet(DJI_Motor_Device wheels[], float x, float y,float w);

#endif //MECANUM_H
