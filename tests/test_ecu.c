
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "ecu_powertrain.h"
#include "ecu_supervisory.h"
#include "can_interface.h"
#include "fault_manager.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL: %s at line %d\n", \
                #condition, __LINE__); \
        return 1; \
    } \
} while (0)

static void SetNormalSensors(PowertrainECU *ecu)
{
    ecu->sensors.motor_speed_rpm = 3500U;
    ecu->sensors.motor_temperature_c = 75.0f;
    ecu->sensors.dc_voltage_v = 400.0f;
    ecu->sensors.dc_current_a = 120.0f;
}

static int ExchangeStatus(
    SupervisoryECU *ecu,
    ECU_State state,
    uint32_t time_ms)
{
    CAN_Message tx, rx;
    ECU_State decoded;

    CHECK(CAN_EncodeStatus(state, &tx));
    CHECK(CAN_Transmit(&tx));
    CHECK(CAN_Receive(&rx));
    CHECK(CAN_DecodeStatus(&rx, &decoded));

    SupervisoryECU_OnStatus(ecu, decoded, time_ms);
    return 0;
}

static int ExchangeDiagnostic(
    SupervisoryECU *ecu,
    FaultInfo fault)
{
    CAN_Message tx, rx;
    FaultInfo decoded;

    CHECK(CAN_EncodeDiagnostic(fault, &tx));
    CHECK(CAN_Transmit(&tx));
    CHECK(CAN_Receive(&rx));
    CHECK(CAN_DecodeDiagnostic(&rx, &decoded));

    SupervisoryECU_OnDiagnostic(ecu, decoded);
    return 0;
}

static int TestNormal(void)
{
    PowertrainECU p;
    SupervisoryECU s;
    PowertrainECU_Init(&p);
    SupervisoryECU_Init(&s);
    SetNormalSensors(&p);

    PowertrainECU_Update(&p);
    CHECK(p.state == ECU_STATE_NORMAL);
    CHECK(ExchangeStatus(&s, p.state, 0U) == 0);
    CHECK(s.state == ECU_STATE_NORMAL);
    CHECK(s.communication_healthy == 1U);
    return 0;
}

static int TestOvertemperature(void)
{
    PowertrainECU p;
    SupervisoryECU s;
    PowertrainECU_Init(&p);
    SupervisoryECU_Init(&s);
    SetNormalSensors(&p);

    p.sensors.motor_temperature_c = 130.0f;
    PowertrainECU_Update(&p);

    FaultInfo fault = FaultManager_Evaluate(
        &p.sensors, p.sensor_valid);

    CHECK(p.state == ECU_STATE_FAULT);
    CHECK(fault.code == FAULT_OVERTEMPERATURE);
    CHECK(fault.severity == SEVERITY_CRITICAL);
    CHECK(ExchangeStatus(&s, p.state, 500U) == 0);
    CHECK(ExchangeDiagnostic(&s, fault) == 0);
    CHECK(s.state == ECU_STATE_FAULT);
    return 0;
}

static int TestInvalidSensor(void)
{
    PowertrainECU p;
    PowertrainECU_Init(&p);
    SetNormalSensors(&p);

    p.sensors.motor_temperature_c = NAN;
    PowertrainECU_Update(&p);

    FaultInfo fault = FaultManager_Evaluate(
        &p.sensors, p.sensor_valid);

    CAN_Message msg;

    CHECK(p.sensor_valid == 0U);
    CHECK(p.state == ECU_STATE_FAULT);
    CHECK(fault.code == FAULT_INVALID_SENSOR);
    CHECK(!CAN_EncodeSensors(&p.sensors, &msg));
    return 0;
}

static int TestTimeout(void)
{
    SupervisoryECU s;
    SupervisoryECU_Init(&s);

    CHECK(ExchangeStatus(&s, ECU_STATE_NORMAL, 0U) == 0);

    SupervisoryECU_Update(&s, 299U);
    CHECK(s.communication_healthy == 1U);

    SupervisoryECU_Update(&s, 300U);
    CHECK(s.communication_healthy == 0U);
    CHECK(s.state == ECU_STATE_FAULT);
    CHECK(s.active_fault.code == FAULT_CAN_TIMEOUT);
    return 0;
}

static int TestFaultRecovery(void)
{
    PowertrainECU p;
    SupervisoryECU s;
    PowertrainECU_Init(&p);
    SupervisoryECU_Init(&s);
    SetNormalSensors(&p);

    p.sensors.motor_temperature_c = 130.0f;
    PowertrainECU_Update(&p);

    CHECK(p.state == ECU_STATE_FAULT);
    CHECK(ExchangeStatus(&s, p.state, 500U) == 0);

    FaultInfo fault = FaultManager_Evaluate(
        &p.sensors, p.sensor_valid);

    CHECK(ExchangeDiagnostic(&s, fault) == 0);
    CHECK(s.state == ECU_STATE_FAULT);

    p.sensors.motor_temperature_c = 75.0f;

    for (int i = 0; i < 5; ++i) {
        PowertrainECU_Update(&p);
    }

    CHECK(p.state == ECU_STATE_NORMAL);

    fault = FaultManager_Evaluate(
        &p.sensors, p.sensor_valid);

    CHECK(ExchangeStatus(&s, p.state, 900U) == 0);
    CHECK(ExchangeDiagnostic(&s, fault) == 0);
    CHECK(s.state == ECU_STATE_NORMAL);
    CHECK(s.active_fault.code == FAULT_NONE);
    return 0;
}

static int TestInvalidCAN(void)
{
    CAN_Message msg = {0};
    ECU_State state;
    FaultInfo fault;
    PowertrainSensorData sensors;

    msg.id = CAN_ID_POWERTRAIN_STATUS;
    msg.dlc = 1U;
    msg.data[0] = 99U;

    CHECK(!CAN_DecodeStatus(&msg, &state));

    msg.data[0] = (uint8_t)ECU_STATE_NORMAL;
    msg.dlc = 8U;
    CHECK(!CAN_DecodeStatus(&msg, &state));

    msg.id = CAN_ID_DIAGNOSTIC;
    msg.dlc = 2U;
    msg.data[0] = 255U;
    msg.data[1] = 2U;
    CHECK(!CAN_DecodeDiagnostic(&msg, &fault));

    msg.id = CAN_ID_SENSOR_DATA;
    msg.dlc = 7U;
    CHECK(!CAN_DecodeSensors(&msg, &sensors));

    return 0;
}

static int TestCommunicationRecovery(void)
{
    SupervisoryECU s;
    SupervisoryECU_Init(&s);

    CHECK(ExchangeStatus(&s, ECU_STATE_NORMAL, 0U) == 0);
    SupervisoryECU_Update(&s, 300U);

    CHECK(s.state == ECU_STATE_FAULT);
    CHECK(s.active_fault.code == FAULT_CAN_TIMEOUT);

    CHECK(ExchangeStatus(&s, ECU_STATE_NORMAL, 350U) == 0);

    CHECK(s.communication_healthy == 1U);
    CHECK(s.active_fault.code == FAULT_NONE);
    CHECK(s.state == ECU_STATE_NORMAL);

    SupervisoryECU_Update(&s, 400U);
    CHECK(s.state == ECU_STATE_NORMAL);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "Usage: ecu_tests <test_name>\n");
        return 1;
    }

    struct TestCase {
        const char *name;
        int (*run)(void);
    };

    static const struct TestCase tests[] = {
        {"normal", TestNormal},
        {"overtemperature", TestOvertemperature},
        {"invalid_sensor", TestInvalidSensor},
        {"timeout", TestTimeout},
        {"fault_recovery", TestFaultRecovery},
        {"invalid_can", TestInvalidCAN},
        {"communication_recovery", TestCommunicationRecovery}
    };

    size_t count = sizeof(tests) / sizeof(tests[0]);

    for (size_t i = 0; i < count; ++i) {
        if (strcmp(argv[1], tests[i].name) == 0) {
            int result = tests[i].run();

            if (result == 0) {
                printf("PASS: %s\n", tests[i].name);
            }

            return result;
        }
    }

    fprintf(stderr, "Unknown test: %s\n", argv[1]);
    return 1;
}
