//
// Created by CYK on 2024/11/30.
//

#ifndef DRV_PUMP_H
#define DRV_PUMP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

typedef enum
{
    CLOSE = 0,
    OPEN_NOT_HOLDING,
    OPEN_HOLDING,
}pump_state_e;

#ifdef __cplusplus
}
#endif

class Pump_Device
{
private:
    float block_current_min;
    float block_current_max;
    float measuring_current;

    uint32_t pump_on_time;
    uint32_t holding_time;

    bool lost_flag;

    pump_state_e state;

public:
    Pump_Device(float block_current_min, float block_current_max);

    bool pump_on_flag;
    bool enable_flag;
    bool holding_flag;

    void Update_Data(float current);
    bool Check_Lost_Flag() const;
    void Set_Ready();
    void Set_Lost();
    bool Check_Holding_Flag();
    void Set_Open();
    void Set_Close();
    void Set_Holding();

    friend class Absorb_Device;
};

#endif //DRV_PUMP_H
