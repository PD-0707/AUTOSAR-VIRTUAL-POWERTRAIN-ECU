
#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <stdint.h>

typedef enum {
    ECU_STATE_INITIALIZING = 0,
    ECU_STATE_NORMAL,
    ECU_STATE_WARNING,
    ECU_STATE_FAULT,
    ECU_STATE_RECOVERY
} ECU_State;

typedef struct {
    uint16_t motor_speed_rpm;
    float motor_temperature_c;
    float dc_voltage_v;
    float dc_current_a;
} PowertrainSensorData;

#endif
