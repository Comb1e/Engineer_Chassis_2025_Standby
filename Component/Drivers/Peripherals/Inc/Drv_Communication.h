//
// Created by CYK on 2024/12/23.
//

#ifndef DRV_COMMUNICATION_H
#define DRV_COMMUNICATION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "Global_CFG.h"

#ifdef __cplusplus
}
#endif

class Communication_Device
{
protected:

public:
    Communication_Device();

    bool connect_flag;
};

extern Communication_Device communication;

#endif //DRV_COMMUNICATION_H
