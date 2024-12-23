//
// Created by CYK on 2024/11/30.
//

#ifndef DRV_ORE_H
#define DRV_ORE_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    NONE = 0,
    WAREHOUSE,//左右矿仓
    SMALL_ISLAND,
    BIG_ISLAND,
    GROUND_MINE
}ore_source_e;

#ifdef __cplusplus
}
#endif

class Ore_Device
{
private:
    ore_source_e source;
public:
    Ore_Device(ore_source_e source);

    ore_source_e Get_Ore_Source();
    void Set_Ore_Source(ore_source_e source);
};

#endif //DRV_ORE_H
