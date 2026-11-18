
cmake_minimum_required(VERSION 3.16)

project(Virtual_Powertrain_ECU C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_library(ecu_core STATIC
    app/ecu_powertrain.c
    app/ecu_supervisory.c
    communication/virtual_can.c
    communication/can_codec.c
    services/scheduler.c
    services/fault_manager.c
)

target_include_directories(ecu_core PUBLIC include)

if(MSVC)
    target_compile_options(ecu_core PRIVATE /W4)
else()
    target_compile_options(ecu_core PRIVATE
        -Wall -Wextra -Wpedantic)
    if(NOT WIN32)
        target_link_libraries(ecu_core PUBLIC m)
    endif()
endif()

add_executable(virtual_ecu app/main.c)
target_link_libraries(virtual_ecu PRIVATE ecu_core)

add_executable(ecu_tests tests/test_ecu.c)
target_link_libraries(ecu_tests PRIVATE ecu_core)

add_test(NAME normal_operation
         COMMAND ecu_tests normal)

add_test(NAME overtemperature
         COMMAND ecu_tests overtemperature)

add_test(NAME invalid_sensor
         COMMAND ecu_tests invalid_sensor)

add_test(NAME communication_timeout
         COMMAND ecu_tests timeout)

add_test(NAME fault_recovery
         COMMAND ecu_tests fault_recovery)

add_test(NAME invalid_can_frame
         COMMAND ecu_tests invalid_can)

add_test(NAME communication_recovery
         COMMAND ecu_tests communication_recovery)
