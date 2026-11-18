
# AUTOSAR-Oriented Virtual Powertrain ECU

A C-based dual-ECU simulation demonstrating
automotive powertrain monitoring, virtual CAN
communication, fault diagnostics, and recovery.

## Architecture

### Powertrain ECU
- Simulates motor speed, temperature, voltage,
  and current measurements.
- Validates sensor data.
- Detects overtemperature and invalid readings.
- Implements NORMAL, WARNING, FAULT,
  and RECOVERY operating states.

### Supervisory ECU
- Receives virtual CAN status and diagnostic frames.
- Monitors powertrain operating state.
- Detects communication timeout.
- Responds to reported faults and recovery.

## Communication

The simulation uses an in-memory CAN mailbox
with application-defined messages:

| CAN ID | Purpose |
|--------|---------|
| 0x180 | Powertrain operating status |
| 0x181 | Encoded sensor measurements |
| 0x182 | Diagnostic information |

The virtual scheduler runs at 10 ms intervals.
Messages are exchanged every 100 ms.
Communication timeout is set to 300 ms.

## Build

Requirements:
- C11-compatible compiler
- CMake
- CTest

```powershell
cmake -S . -B build
cmake --build build
.\build\virtual_ecu.exe
```

## Automated Tests

Run:

```powershell
ctest --test-dir build --output-on-failure
```

Seven tests cover:
1. Normal operation
2. Overtemperature detection
3. Invalid sensor handling
4. Communication timeout
5. Fault recovery
6. Invalid CAN frame rejection
7. Communication recovery

Latest local validation: 7/7 tests passed.

## Technology

Embedded C (C11), CMake, CTest,
virtual CAN, finite-state machines,
fault diagnostics, deterministic simulation.

## Scope

This is an AUTOSAR-oriented educational simulation.
It does not implement a certified AUTOSAR Classic
BSW stack, RTE, physical CAN controller,
or production automotive safety mechanisms.
