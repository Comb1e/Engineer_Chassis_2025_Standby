//
// Created by CYK on 2024/12/14.
//

#ifndef DRV_BACK_H
#define DRV_BACK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "Global_CFG.h"

/**
 * @brief 矿石吸取位置的不同需要加的偏置
 */
#define BIG_ISLAND_BACK_SUCKER_X_OFFSET  (-0.0f)
#define SMALL_ISLAND_BACK_SUCKER_X_OFFSET (0.0f)
#define SMALL_ISLAND_BACK_FRONT_SUCKER_X_OFFSET (-20.0f)

#define GROUND_MINE_BACK_SUCKER_X_OFFSET  (10.0f)

/**
 * @brief 放矿到左右吸盘的位置和速度,先xyz,再ryp
 */
#define LEFT_SUCKER_POSITION_PRE_X     (342.0f)
#define LEFT_SUCKER_POSITION_X         (342.0f)
#define LEFT_SUCKER_POSITION_PRE_Y     (220.0f)
#define LEFT_SUCKER_POSITION_Y         (222.0f)
#define LEFT_SUCKER_POSITION_PRE_Z     (300.0f)
#define LEFT_SUCKER_POSITION_Z         (50.f)
#define LEFT_SUCKER_POSITION_PITCH     (-3.0f)
#define LEFT_SUCKER_POSITION_YAW       (97.0f)
#define LEFT_SUCKER_POSITION_ROLL      (0.0f)
#define LEFT_SUCKER_POSITION_ARM_YAW   (22.0f)
#define LEFT_SUCKER_POSITION_ARM_PITCH (0.0f)

#define RIGHT_SUCKER_POSITION_PRE_X  (342.0f)
#define RIGHT_SUCKER_POSITION_X      (342.0f)
#define RIGHT_SUCKER_POSITION_PRE_Y      (-80.0f)
#define RIGHT_SUCKER_POSITION_Y      (-40.0f)
#define RIGHT_SUCKER_POSITION_PRE_Z      (200.0f)
#define RIGHT_SUCKER_POSITION_Z          (50.f)
#define RIGHT_SUCKER_POSITION_PITCH         (-3.0f)
#define RIGHT_SUCKER_POSITION_YAW      (-97.0f)
#define RIGHT_SUCKER_POSITION_ROLL      (0.0f)
#define RIGHT_SUCKER_POSITION_ARM_YAW      (-22.0f)
#define RIGHT_SUCKER_POSITION_ARM_PITCH (0.0f)

#define ARM_SUCKER_POSITION_X         (550.0f)
#define ARM_SUCKER_POSITION_Y         (70.0f)
#define ARM_SUCKER_POSITION_Z_PRE     (70.f)
#define ARM_SUCKER_POSITION_Z         (0.0f)
#define ARM_SUCKER_POSITION_PITCH     (90.0f)
#define ARM_SUCKER_POSITION_ARM_YAW   (0.0f)
#define ARM_SUCKER_POSITION_ARM_PITCH (0.0f)
#define ARM_SUCKER_POSITION_YAW       (0.0f)
#define ARM_SUCKER_POSITION_ROLL      (0.0f)

#define SIDE_SUCKER_POSITION_XYZ_VEL  (1.0f)
#define SIDE_SUCKER_POSITION_RYP_VEL  (0.15f)

#define ARM_SUCKER_POSITION_XYZ_VEL   (1.0f)
#define ARM_SUCKER_POSITION_RYP_VEL   (0.15f)

#define BACK_HOME_POSITION_ARM_PITCH (0.0f)

#define BACK_HOME_POSITION_ARM_YAW (0.0f)
#define BACK_HOME_POSITION_YAW      (0.0f)
#define BACK_HOME_POSITION_PITCH    (0.0f)
#define BACK_HOME_POSITION_ROLL     (0.0f)
#define BACK_HOME_POSITION_X        (400.f)
#define BACK_HOME_POSITION_Y        (70.f)
#define BACK_HOME_POSITION_Z        (20.f)

#define BACK_HOME_POSITION_X_VEL    (1.0f)
#define BACK_HOME_POSITION_Z_VEL    (1.0f)
#define BACK_HOME_POSITION_Y_VEL    (1.0f)
#define BACK_HOME_POSITION_RYP_VEL  (0.15f)
#define BACK_HOME_POSITION_ARM_YAW_VEL  (0.2f)

#ifdef __cplusplus
}
#endif

#endif //DRV_BACK_H
