
#ifndef ECU_SUPERVISORY_H
#define ECU_SUPERVISORY_H

#include "system_types.h"
#include "fault_manager.h"
#include <stdint.h>

typedef struct {
    ECU_State state;
    uint8_t communication_healthy;
    uint8_t received_once;
    uint32_t last_rx_ms;
    ECU_State remote_powertrain_state;
    FaultInfo active_fault;
} SupervisoryECU;

void SupervisoryECU_Init(SupervisoryECU *ecu);

void SupervisoryECU_Update(
    SupervisoryECU *ecu,
    uint32_t current_time_ms
);

void SupervisoryECU_OnMessage(
    SupervisoryECU *ecu,
    uint32_t current_time_ms
);

void SupervisoryECU_OnStatus(
    SupervisoryECU *ecu,
    ECU_State remote_state,
    uint32_t current_time_ms
);

void SupervisoryECU_OnDiagnostic(
    SupervisoryECU *ecu,
    FaultInfo fault
);

#endif
