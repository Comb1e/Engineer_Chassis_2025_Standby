//
// Created by CYK on 2024/11/30.
//

#include "Drv_Ore.h"

#include "Drv_Absorb.h"

Ore_Device::Ore_Device(ore_source_e source)
{
    this->source = source;
}

ore_source_e Ore_Device::Get_Ore_Source()
{
    return this->source;
}

void Ore_Device::Set_Ore_Source(ore_source_e source)
{
    this->source = source;
}
