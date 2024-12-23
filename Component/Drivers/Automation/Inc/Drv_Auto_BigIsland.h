//
// Created by CYK on 2024/12/18.
//

#ifndef DRV_AUTO_BIGISLAND_H
#define DRV_AUTO_BIGISLAND_H

#include "Drv_Robot.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "Global_CFG.h"

/*----------Pre----------*/
#define BIG_ISLAND_PRE_RIGHT_POSITION_ARM_YAW        (15.0f)
#define BIG_ISLAND_PRE_CENTER_POSITION_ARM_YAW        (0.0f)
#define BIG_ISLAND_PRE_LEFT_POSITION_ARM_YAW        (15.0f)
#define BIG_ISLAND_PRE_POSITION_X                   (400.0f)
#define BIG_ISLAND_PRE_POSITION_Y_CENTER                   (70.0f)
#define BIG_ISLAND_PRE_POSITION_Y_LEFT                   (60.0f)
#define BIG_ISLAND_PRE_POSITION_Y_RIGHT                   (60.0f)
#define BIG_ISLAND_PRE_POSITION_Z                   (250.0f)
#define BIG_ISLAND_PRE_POSITION_ROLL                (0.0f)
#define BIG_ISLAND_PRE_POSITION_PITCH               (0.0f)
#define BIG_ISLAND_PRE_POSITION_ARM_PITCH               (0.0f)
#define BIG_ISLAND_PRE_RIGHT_POSITION_YAW                 (15.0f)
#define BIG_ISLAND_PRE_CENTER_POSITION_YAW                 (0.0f)
#define BIG_ISLAND_PRE_LEFT_POSITION_YAW                 (15.0f)

#define BIG_ISLAND_PRE_POSITION_XYZ_VEL             (1.0f)
#define BIG_ISLAND_PRE_POSITION_RYP_VEL             (0.2f)

/*----------1----------*/
#define BIG_ISLAND_1_DISTANCE_CENTER_1                       (220.0f)
#define BIG_ISLAND_1_DISTANCE_LEFT_OR_RIGHT_1                    (0.0f)

#define BIG_ISLAND_1_DISTANCE_CENTER_2                       (230.0f)
#define BIG_ISLAND_1_DISTANCE_LEFT_OR_RIGHT_2                       (150.0f)

#define BIG_ISLAND_1_VEL                            (1.0f)

#define BIG_ISLAND_1_PITCH                           (0.0f)
#define BIG_ISLAND_1_PITCH_VEl                       (0.2f)

/*----------Touching----------*/
#define BIG_ISLAND_TOUCHING_DELTA_DISTANCE                       (1.0f)
#define BIG_ISLAND_TOUCHING_VEL                            (0.2f)

/*----------ADJUST_1----------*/
#define BIG_ISLAND_ADJUST_1_PITCH                       (-10.0f)
#define BIG_ISLAND_ADJUST_1_Z                           (270.0f)
#define BIG_ISLAND_ADJUST_1_PITCH_VEl                   (0.2f)
#define BIG_ISLAND_ADJUST_1_Z_VEl                           (1.0f)

/*----------2----------*/
#define BIG_ISLAND_2_DISTANCE_CENTER                           (-220.0f)
#define BIG_ISLAND_2_DISTANCE_LEFT_OR_RIGHT                    (-80.0f)

#define BIG_ISLAND_2_VEL                                 (1.0f)

/*----------ADJUST_2----------*/
#define BIG_ISLAND_ADJUST_2_CENTER_YAW                       (0.0f)
#define BIG_ISLAND_ADJUST_2_LEFT_OR_RIGHT_YAW                       (15.0f)

#define BIG_ISLAND_ADJUST_2_YAW_VEl                   (0.1f)

/*----------3----------*/
#define BIG_ISLAND_3_DISTANCE_CENTER_1                           (-330.0f)
#define BIG_ISLAND_3_DISTANCE_LEFT_OR_RIGHT_1                           (-280.0f)

#define BIG_ISLAND_3_VEL_1                                (0.8f)

#define BIG_ISLAND_3_DISTANCE_CENTER_2                           (-100.0f)
#define BIG_ISLAND_3_DISTANCE_LEFT_OR_RIGHT_2            (-260.0f)

#define BIG_ISLAND_3_VEL_2                                (1.0f)

/*----------Pre_Back----------*/
#define BIG_ISLAND_PRE_BACK_YAW                         (0.0f)
#define BIG_ISLAND_PRE_BACK_ARM_YAW                     (0.0f)
#define BIG_ISLAND_PRE_BACK_PITCH                       (0.0f)

#define BIG_ISLAND_PRE_BACK_RYP_VEL                     (0.2f)

#ifdef __cplusplus
}
#endif

#endif //DRV_AUTO_BIGISLAND_H
