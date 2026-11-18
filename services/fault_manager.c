
#include "fault_manager.h"
#include <stddef.h>

FaultInfo FaultManager_Evaluate(
    const PowertrainSensorData *sensors,
    uint8_t sensor_valid)
{
    FaultInfo result = {FAULT_NONE, SEVERITY_NONE};

    if (sensors == NULL || !sensor_valid) {
        result.code = FAULT_INVALID_SENSOR;
        result.severity = SEVERITY_CRITICAL;
        return result;
    }

    if (sensors->motor_temperature_c > 120.0f) {
        result.code = FAULT_OVERTEMPERATURE;
        result.severity = SEVERITY_CRITICAL;
    }
    else if (sensors->motor_temperature_c >= 100.0f) {
        result.code = FAULT_OVERTEMPERATURE;
        result.severity = SEVERITY_WARNING;
    }

    return result;
}

const char *FaultManager_GetName(FaultCode code)
{
    switch (code) {
        case FAULT_NONE:
            return "NONE";
        case FAULT_OVERTEMPERATURE:
            return "OVERTEMPERATURE";
        case FAULT_INVALID_SENSOR:
            return "INVALID_SENSOR";
        case FAULT_CAN_TIMEOUT:
            return "CAN_TIMEOUT";
        default:
            return "UNKNOWN";
    }
}
