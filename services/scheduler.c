
#include <stdint.h>

#define SCHEDULER_TICK_MS 10U

static uint32_t simulation_time_ms = 0U;

void Scheduler_Reset(void)
{
    simulation_time_ms = 0U;
}

void Scheduler_Tick(void)
{
    simulation_time_ms += SCHEDULER_TICK_MS;
}

uint32_t Scheduler_GetTime(void)
{
    return simulation_time_ms;
}
