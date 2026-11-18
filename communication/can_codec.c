
#include "can_interface.h"
#include <stddef.h>
#include <math.h>

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8);
}

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)(
        (uint16_t)data[0] |
        ((uint16_t)data[1] << 8)
    );
}

bool CAN_EncodeSensors(
    const PowertrainSensorData *sensors,
    CAN_Message *message)
{
    if (sensors == NULL || message == NULL) {
        return false;
    }

    if (sensors->motor_speed_rpm > 12000U ||
        !isfinite(sensors->motor_temperature_c) ||
        !isfinite(sensors->dc_voltage_v) ||
        !isfinite(sensors->dc_current_a) ||
        sensors->motor_temperature_c < -40.0f ||
        sensors->motor_temperature_c > 180.0f ||
        sensors->dc_voltage_v < 0.0f ||
        sensors->dc_voltage_v > 800.0f ||
        sensors->dc_current_a < -500.0f ||
        sensors->dc_current_a > 500.0f) {
        return false;
    }

    int16_t temperature =
        (int16_t)lroundf(sensors->motor_temperature_c * 10.0f);

    uint16_t voltage =
        (uint16_t)lroundf(sensors->dc_voltage_v * 10.0f);

    int16_t current =
        (int16_t)lroundf(sensors->dc_current_a * 10.0f);

    message->id = CAN_ID_SENSOR_DATA;
    message->dlc = 8U;

    write_u16(&message->data[0],
              sensors->motor_speed_rpm);

    write_u16(&message->data[2],
              (uint16_t)temperature);

    write_u16(&message->data[4], voltage);

    write_u16(&message->data[6],
              (uint16_t)current);

    return true;
}

bool CAN_DecodeSensors(
    const CAN_Message *message,
    PowertrainSensorData *sensors)
{
    if (message == NULL || sensors == NULL ||
        message->id != CAN_ID_SENSOR_DATA ||
        message->dlc != 8U) {
        return false;
    }

    PowertrainSensorData decoded;

    decoded.motor_speed_rpm =
        read_u16(&message->data[0]);

    decoded.motor_temperature_c =
        (float)(int16_t)read_u16(&message->data[2]) / 10.0f;

    decoded.dc_voltage_v =
        (float)read_u16(&message->data[4]) / 10.0f;

    decoded.dc_current_a =
        (float)(int16_t)read_u16(&message->data[6]) / 10.0f;

    if (decoded.motor_speed_rpm > 12000U ||
        decoded.motor_temperature_c < -40.0f ||
        decoded.motor_temperature_c > 180.0f ||
        decoded.dc_voltage_v > 800.0f ||
        decoded.dc_current_a < -500.0f ||
        decoded.dc_current_a > 500.0f) {
        return false;
    }

    *sensors = decoded;
    return true;
}

bool CAN_EncodeStatus(
    ECU_State state,
    CAN_Message *message)
{
    if (message == NULL ||
        state < ECU_STATE_INITIALIZING ||
        state > ECU_STATE_RECOVERY) {
        return false;
    }

    message->id = CAN_ID_POWERTRAIN_STATUS;
    message->dlc = 1U;
    message->data[0] = (uint8_t)state;

    return true;
}

bool CAN_DecodeStatus(
    const CAN_Message *message,
    ECU_State *state)
{
    if (message == NULL || state == NULL ||
        message->id != CAN_ID_POWERTRAIN_STATUS ||
        message->dlc != 1U ||
        message->data[0] > (uint8_t)ECU_STATE_RECOVERY) {
        return false;
    }

    *state = (ECU_State)message->data[0];
    return true;
}

bool CAN_EncodeDiagnostic(
    FaultInfo fault,
    CAN_Message *message)
{
    if (message == NULL ||
        fault.code < FAULT_NONE ||
        fault.code > FAULT_CAN_TIMEOUT ||
        fault.severity < SEVERITY_NONE ||
        fault.severity > SEVERITY_CRITICAL) {
        return false;
    }

    message->id = CAN_ID_DIAGNOSTIC;
    message->dlc = 2U;
    message->data[0] = (uint8_t)fault.code;
    message->data[1] = (uint8_t)fault.severity;

    return true;
}

bool CAN_DecodeDiagnostic(
    const CAN_Message *message,
    FaultInfo *fault)
{
    if (message == NULL || fault == NULL ||
        message->id != CAN_ID_DIAGNOSTIC ||
        message->dlc != 2U ||
        message->data[0] > (uint8_t)FAULT_CAN_TIMEOUT ||
        message->data[1] > (uint8_t)SEVERITY_CRITICAL) {
        return false;
    }

    fault->code = (FaultCode)message->data[0];
    fault->severity = (FaultSeverity)message->data[1];

    return true;
}
