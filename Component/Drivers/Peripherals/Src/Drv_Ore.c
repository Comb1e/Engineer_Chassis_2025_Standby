//
// Created by CYK on 2024/11/22.
//

#include "Drv_Ore.h"

ore_t ore;

void Ore_Init(ore_t *ore,enum ore_source_e source)
{
    ore->ore_source = source;
}

enum ore_source_e Get_Ore_Source(ore_t *ore)
{
    return ore->ore_source;
}

void Set_Ore_Source(ore_t *ore,enum ore_source_e source)
{
    ore->ore_source = source;
}