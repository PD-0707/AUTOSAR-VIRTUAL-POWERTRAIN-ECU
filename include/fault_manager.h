
#ifndef FAULT_MANAGER_H
#define FAULT_MANAGER_H

#include <stdint.h>
#include "system_types.h"

typedef enum {
    FAULT_NONE = 0,
    FAULT_OVERTEMPERATURE = 1,
    FAULT_INVALID_SENSOR = 2,
    FAULT_CAN_TIMEOUT = 3
} FaultCode;

typedef enum {
    SEVERITY_NONE = 0,
    SEVERITY_WARNING = 1,
    SEVERITY_CRITICAL = 2
} FaultSeverity;

typedef struct {
    FaultCode code;
    FaultSeverity severity;
} FaultInfo;

FaultInfo FaultManager_Evaluate(
    const PowertrainSensorData *sensors,
    uint8_t sensor_valid
);

const char *FaultManager_GetName(FaultCode code);

#endif
