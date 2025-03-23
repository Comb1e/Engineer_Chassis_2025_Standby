//
// Created by CYK on 2024/12/19.
//

#ifndef DRV_AUTO_SMALLISLAND_OR_GROUNDMINE_H
#define DRV_AUTO_SMALLISLAND_OR_GROUNDMINE_H

#include "Drv_Robot.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "Global_CFG.h"

/*----------Pre----------*/
#define GROUND_MINE_PRE_POSITION_X      (500.0f)
#define GROUND_MINE_PRE_POSITION_Y      (70.0f)
#define GROUND_MINE_PRE_POSITION_Z      (20.0f)
#define GROUND_MINE_PRE_POSITION_ARM_YAW      (0.0f)
#define GROUND_MINE_PRE_POSITION_ARM_PITCH      (0.0f)
#define GROUND_MINE_PRE_POSITION_YAW      (0.0f)
#define GROUND_MINE_PRE_POSITION_ROLL      (0.0f)
#define GROUND_MINE_PRE_POSITION_PITCH      (0.0f)

#define GROUND_MINE_PRE_POSITION_XYZ_VEL    (0.8f)
#define GROUND_MINE_PRE_POSITION_RYP_VEL    (0.1f)

/*----------1----------*/
#define GROUND_MINE_1_POSITION_X      (720.0f)
#define GROUND_MINE_1_POSITION_Z      (-90.0f)

#define GROUND_MINE_1_POSITION_PITCH      (91.0f)
#define GROUND_MINE_1_POSITION_ARM_PITCH  (90.0f)

#define GROUND_MINE_1_POSITION_XYZ_VEL    (0.8f)
#define GROUND_MINE_1_POSITION_RYP_VEL    (0.1f)

/*----------Touching----------*/
#define GROUND_MINE_TOUCHING_POSITION_Z      (-180.0f)
#define GROUND_MINE_TOUCHING_POSITION_YAW (0.0f)

#define GROUND_MINE_TOUCHING_POSITION_Z_VEL    (0.1f)

    /*----------Lay----------*/
#define SMALLISLAND_LAY_POSITION_XYZ_VEL    (0.8f)
#define SMALLISLAND_LAY_POSITION_RYP_VEL    (0.2f)
/*----------Pre_Back----------*/
#define GROUND_MINE_PRE_BACK_POSITION_Z      (200.0f)
#define GROUND_MINE_PRE_BACK_POSITION_X      (520.0f)
#define GROUND_MINE_PRE_BACK_POSITION_ARM_PITCH (0.0f)
#define GROUND_MINE_PRE_BACK_POSITION_PITCH (0.0f)
#define GROUND_MINE_PRE_BACK_POSITION_YAW (0.0)
#define GROUND_MINE_PRE_BACK_POSITION_ROLL (0.0f)

#define GROUND_MINE_PRE_BACK_POSITION_XYZ_VEL    (0.8f)
#define GROUND_MINE_PRE_BACK_POSITION_RYP_VEL    (0.1f)

#ifdef __cplusplus
}
#endif

#endif //DRV_AUTO_SMALLISLAND_OR_GROUNDMINE_H
