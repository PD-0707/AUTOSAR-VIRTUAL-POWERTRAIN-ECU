
#include "ecu_powertrain.h"
#include <math.h>
#include <stddef.h>

#define WARNING_TEMP_C 100.0f
#define CRITICAL_TEMP_C 120.0f
#define RECOVERY_TICKS 5U

void PowertrainECU_Init(PowertrainECU *ecu)
{
    if (ecu == NULL) return;

    ecu->state = ECU_STATE_INITIALIZING;
    ecu->healthy_ticks = 0U;
    ecu->sensor_valid = 0U;

    ecu->sensors.motor_speed_rpm = 0U;
    ecu->sensors.motor_temperature_c = 0.0f;
    ecu->sensors.dc_voltage_v = 0.0f;
    ecu->sensors.dc_current_a = 0.0f;
}

void PowertrainECU_Update(PowertrainECU *ecu)
{
    if (ecu == NULL) return;

    PowertrainSensorData *s = &ecu->sensors;

    ecu->sensor_valid =
        s->motor_speed_rpm <= 12000U &&
        isfinite(s->motor_temperature_c) &&
        isfinite(s->dc_voltage_v) &&
        isfinite(s->dc_current_a) &&
        s->motor_temperature_c >= -40.0f &&
        s->motor_temperature_c <= 180.0f &&
        s->dc_voltage_v >= 0.0f &&
        s->dc_voltage_v <= 800.0f &&
        s->dc_current_a >= -500.0f &&
        s->dc_current_a <= 500.0f;

    if (!ecu->sensor_valid ||
        s->motor_temperature_c > CRITICAL_TEMP_C) {
        ecu->state = ECU_STATE_FAULT;
        ecu->healthy_ticks = 0U;
        return;
    }

    if (s->motor_temperature_c >= WARNING_TEMP_C) {
        ecu->state = ECU_STATE_WARNING;
        ecu->healthy_ticks = 0U;
        return;
    }

    if (ecu->state == ECU_STATE_FAULT ||
        ecu->state == ECU_STATE_RECOVERY) {
        ecu->state = ECU_STATE_RECOVERY;
        ecu->healthy_ticks++;

        if (ecu->healthy_ticks >= RECOVERY_TICKS) {
            ecu->state = ECU_STATE_NORMAL;
            ecu->healthy_ticks = 0U;
        }
        return;
    }

    ecu->state = ECU_STATE_NORMAL;
}
