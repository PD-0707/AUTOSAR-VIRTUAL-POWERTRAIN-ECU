
#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include "system_types.h"

#define CAN_ID_POWERTRAIN_STATUS 0x180U
#define CAN_ID_SENSOR_DATA       0x181U
#define CAN_ID_DIAGNOSTIC        0x7DFU
#define CAN_MAX_DATA_LENGTH 8U

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[CAN_MAX_DATA_LENGTH];
} CAN_Message;

bool CAN_Transmit(const CAN_Message *message);
bool CAN_Receive(CAN_Message *message);

bool CAN_EncodeSensors(
    const PowertrainSensorData *sensors,
    CAN_Message *message
);

bool CAN_DecodeSensors(
    const CAN_Message *message,
    PowertrainSensorData *sensors
);

#include "fault_manager.h"

bool CAN_EncodeStatus(
    ECU_State state,
    CAN_Message *message
);

bool CAN_DecodeStatus(
    const CAN_Message *message,
    ECU_State *state
);

bool CAN_EncodeDiagnostic(
    FaultInfo fault,
    CAN_Message *message
);

bool CAN_DecodeDiagnostic(
    const CAN_Message *message,
    FaultInfo *fault
);

#endif
