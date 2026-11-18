
#include "ecu_supervisory.h"
#include <stddef.h>

#define CAN_TIMEOUT_MS 300U

static void RefreshState(SupervisoryECU *ecu)
{
    if (!ecu->communication_healthy) {
        ecu->state = ECU_STATE_FAULT;
    }
    else if (ecu->remote_powertrain_state == ECU_STATE_FAULT ||
             ecu->active_fault.severity == SEVERITY_CRITICAL) {
        ecu->state = ECU_STATE_FAULT;
    }
    else if (ecu->remote_powertrain_state == ECU_STATE_WARNING ||
             ecu->active_fault.severity == SEVERITY_WARNING) {
        ecu->state = ECU_STATE_WARNING;
    }
    else {
        ecu->state = ecu->remote_powertrain_state;
    }
}

void SupervisoryECU_Init(SupervisoryECU *ecu)
{
    if (ecu == NULL) return;

    ecu->state = ECU_STATE_INITIALIZING;
    ecu->communication_healthy = 0U;
    ecu->received_once = 0U;
    ecu->last_rx_ms = 0U;
    ecu->remote_powertrain_state = ECU_STATE_INITIALIZING;
    ecu->active_fault.code = FAULT_NONE;
    ecu->active_fault.severity = SEVERITY_NONE;
}

void SupervisoryECU_OnMessage(
    SupervisoryECU *ecu,
    uint32_t current_time_ms)
{
    if (ecu == NULL) return;

    ecu->last_rx_ms = current_time_ms;
    ecu->received_once = 1U;
    ecu->communication_healthy = 1U;

    if (ecu->active_fault.code == FAULT_CAN_TIMEOUT) {
        ecu->active_fault.code = FAULT_NONE;
        ecu->active_fault.severity = SEVERITY_NONE;
    }

    RefreshState(ecu);
}

void SupervisoryECU_OnStatus(
    SupervisoryECU *ecu,
    ECU_State remote_state,
    uint32_t current_time_ms)
{
    if (ecu == NULL) return;

    if (remote_state < ECU_STATE_INITIALIZING ||
        remote_state > ECU_STATE_RECOVERY) {
        return;
    }

    ecu->remote_powertrain_state = remote_state;
    SupervisoryECU_OnMessage(ecu, current_time_ms);
}

void SupervisoryECU_OnDiagnostic(
    SupervisoryECU *ecu,
    FaultInfo fault)
{
    if (ecu == NULL) return;

    ecu->active_fault = fault;
    RefreshState(ecu);
}

void SupervisoryECU_Update(
    SupervisoryECU *ecu,
    uint32_t current_time_ms)
{
    if (ecu == NULL) return;

    if ((uint32_t)(current_time_ms - ecu->last_rx_ms)
        >= CAN_TIMEOUT_MS) {

        ecu->communication_healthy = 0U;
        ecu->active_fault.code = FAULT_CAN_TIMEOUT;
        ecu->active_fault.severity = SEVERITY_CRITICAL;
    }

    RefreshState(ecu);
}
