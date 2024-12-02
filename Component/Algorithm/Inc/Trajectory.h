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

#endif //TRAJECTORY_H
