//
// Created by CYK on 2024/11/30.
//

#ifndef DRV_ABSORB_H
#define DRV_ABSORB_H

#include "BSP_CAN.h"
#include "Drv_Pump.h"
#include "Drv_Ore.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "Global_CFG.h"

#define PUMP_CAN        (&hcan1)

#define ABSORB_CAN_TX_STDID  (0x502)
#define ABSORB_CAN_RX_STDID  (0x501)

#define ORE_NUM_MAX 3
#define ARM_SUCKER_BLOCK_CURRENT_MIN   1850.0f
#define ARM_SUCKER_BLOCK_CURRENT_MAX   1940.0f

#define LEFT_SUCKER_BLOCK_CURRENT_MIN   1850.0f
#define LEFT_SUCKER_BLOCK_CURRENT_MAX   1950.0f

#define RIGHT_SUCKER_BLOCK_CURRENT_MIN   1850.0f
#define RIGHT_SUCKER_BLOCK_CURRENT_MAX   1950.0f

typedef enum
{
    LEFT_SUCKER = 0,
    RIGHT_SUCKER,
    ARM_SUCKER,
    ORE_STORE_FULL = -1,
    ARM_SUCKER_NO_ORE = -2,
    ORE_STORE_NONE = -3
}sucker_e;

#pragma pack(1)
typedef struct
{
    uint8_t arm_sucker_pump_on_flag: 1;
    uint8_t left_sucker_pump_on_flag: 1;
    uint8_t right_sucker_pump_on_flag: 1;
    uint8_t reverse_1: 5;
    uint8_t reverse_2: 8;
    uint8_t reverse_3: 8;
    uint8_t reverse_4: 8;
    uint8_t reverse_5: 8;
    uint8_t reverse_6: 8;
    uint8_t reverse_7: 8;
    uint8_t reverse_8: 8;
}absorb_can_tx_data_t;
#pragma pack(0)

typedef struct
{
    uint16_t arm_sucker_current;
    uint16_t left_sucker_current;
    uint16_t right_sucker_current;
}absorb_can_rx_data_t;

#ifdef __cplusplus
}
#endif

class Absorb_Device
{
private:
    can_device_t can_device;

public:
    Absorb_Device();

    Pump_Device sucker[ORE_NUM_MAX];

    Ore_Device holding_ore;

    bool lost_flag;
    bool ready_flag;
    bool enable_flag;

    bool open_left_pump_flag;
    bool open_right_pump_flag;
    bool open_arm_pump_flag;

    uint8_t ore_num;

    absorb_can_tx_data_t tx_data;
    absorb_can_rx_data_t rx_data;

    void Init(CAN_HandleTypeDef *hcan, uint32_t rx_stdid, uint32_t tx_stdid, osSemaphoreId_t rx_sem);
    void Update_Data();
    void Update_Ready();
    void Check_Pump_MCU_For_Loss();
    void Update_Ore_Num();
    void Update_MSG();
    void Set_Sucker_Open(sucker_e sucker);
    void Set_Sucker_Close(sucker_e sucker);
    Ore_Device* Get_Ore_State();
    uint8_t Get_Ore_Num() const;
    sucker_e Find_Ore();
    pump_state_e Get_Sucker_State(sucker_e sucker);
    sucker_e Find_To_Store();
    bool Check_Sucker_Holding(sucker_e pos);
    void Set_Sucker_Holding();
    bool Check_Enable();
    void Update_Pump_Open();

    friend void Absorb_RX_Callback(can_device_t *can_device,uint8_t *rx_data);

    friend class Pump_Device;
};

void Absorb_RX_Callback(can_device_t *can_device,uint8_t *rx_data);

extern Absorb_Device absorb;

#endif //DRV_ABSORB_H

