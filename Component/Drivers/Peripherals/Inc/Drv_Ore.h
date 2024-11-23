//
// Created by CYK on 2024/11/22.
//

#ifndef DRV_ORE_H
#define DRV_ORE_H

enum ore_source_e
{
    NONE = 0,
    WAREHOUSE,//左右矿仓
    SMALL_ISLAND,
    BIG_ISLAND,
    GROUND_MINE
};

typedef struct
{
    enum ore_source_e ore_source;
}ore_t;

void Ore_Init(ore_t *ore,enum ore_source_e source);
enum ore_source_e Get_Ore_Source(ore_t *ore);
void Set_Ore_Source(ore_t *ore,enum ore_source_e source);

extern ore_t ore;

#endif //DRV_ORE_H

