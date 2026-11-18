
#ifndef ECU_POWERTRAIN_H
#define ECU_POWERTRAIN_H

#include "system_types.h"
#include <stdint.h>

typedef struct {
    ECU_State state;
    PowertrainSensorData sensors;
    uint32_t healthy_ticks;
    uint8_t sensor_valid;
} PowertrainECU;

void PowertrainECU_Init(PowertrainECU *ecu);
void PowertrainECU_Update(PowertrainECU *ecu);

#endif
