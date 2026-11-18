
#include <stdio.h>
#include <stdint.h>

#include "ecu_powertrain.h"
#include "ecu_supervisory.h"
#include "can_interface.h"
#include "fault_manager.h"
#include "scheduler.h"

static const char *state_name(ECU_State state)
{
    switch (state) {
        case ECU_STATE_INITIALIZING: return "INITIALIZING";
        case ECU_STATE_NORMAL: return "NORMAL";
        case ECU_STATE_WARNING: return "WARNING";
        case ECU_STATE_FAULT: return "FAULT";
        case ECU_STATE_RECOVERY: return "RECOVERY";
        default: return "UNKNOWN";
    }
}

static int SendStatus(
    SupervisoryECU *supervisory,
    ECU_State state,
    uint32_t time_ms)
{
    CAN_Message tx, rx;
    ECU_State decoded;

    if (!CAN_EncodeStatus(state, &tx) ||
        !CAN_Transmit(&tx) ||
        !CAN_Receive(&rx) ||
        !CAN_DecodeStatus(&rx, &decoded)) {
        return 0;
    }

    SupervisoryECU_OnStatus(
        supervisory, decoded, time_ms);

    return 1;
}

static int SendDiagnostic(
    SupervisoryECU *supervisory,
    FaultInfo fault)
{
    CAN_Message tx, rx;
    FaultInfo decoded;

    if (!CAN_EncodeDiagnostic(fault, &tx) ||
        !CAN_Transmit(&tx) ||
        !CAN_Receive(&rx) ||
        !CAN_DecodeDiagnostic(&rx, &decoded)) {
        return 0;
    }

    SupervisoryECU_OnDiagnostic(
        supervisory, decoded);

    return 1;
}

static int SendSensors(
    PowertrainECU *powertrain)
{
    CAN_Message tx, rx;
    PowertrainSensorData decoded;

    return CAN_EncodeSensors(&powertrain->sensors, &tx) &&
           CAN_Transmit(&tx) &&
           CAN_Receive(&rx) &&
           CAN_DecodeSensors(&rx, &decoded);
}

int main(void)
{
    PowertrainECU powertrain;
    SupervisoryECU supervisory;

    PowertrainECU_Init(&powertrain);
    SupervisoryECU_Init(&supervisory);
    Scheduler_Reset();

    powertrain.sensors.motor_speed_rpm = 3500U;
    powertrain.sensors.motor_temperature_c = 75.0f;
    powertrain.sensors.dc_voltage_v = 400.0f;
    powertrain.sensors.dc_current_a = 120.0f;

    printf("Phase 4: ECU Fault Diagnostics\n");
    printf("---------------------------------\n");

    for (uint32_t step = 0U; step < 150U; step++) {
        uint32_t time_ms = Scheduler_GetTime();

        if (time_ms == 500U) {
            powertrain.sensors.motor_temperature_c = 130.0f;
            printf("[%u ms] Overtemperature injected\n",
                   (unsigned int)time_ms);
        }

        if (time_ms == 800U) {
            powertrain.sensors.motor_temperature_c = 75.0f;
            printf("[%u ms] Temperature restored\n",
                   (unsigned int)time_ms);
        }

        PowertrainECU_Update(&powertrain);

        FaultInfo fault = FaultManager_Evaluate(
            &powertrain.sensors,
            powertrain.sensor_valid
        );

        if ((time_ms % 100U) == 0U &&
            time_ms < 1000U) {

            if (!SendStatus(
                    &supervisory,
                    powertrain.state,
                    time_ms) ||
                !SendDiagnostic(
                    &supervisory,
                    fault) ||
                !SendSensors(&powertrain)) {

                printf("CAN exchange failed at %u ms\n",
                       (unsigned int)time_ms);
                return 1;
            }
        }

        SupervisoryECU_Update(&supervisory, time_ms);

        if ((time_ms % 100U) == 0U) {
            printf(
                "[%4u ms] Powertrain=%-10s "
                "Supervisory=%-10s "
                "Fault=%-17s CAN=%s\n",
                (unsigned int)time_ms,
                state_name(powertrain.state),
                state_name(supervisory.state),
                FaultManager_GetName(
                    supervisory.active_fault.code),
                supervisory.communication_healthy ?
                    "OK" : "TIMEOUT"
            );
        }

        Scheduler_Tick();
    }

    return 0;
}
