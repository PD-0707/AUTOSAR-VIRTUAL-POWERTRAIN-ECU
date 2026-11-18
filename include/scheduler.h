
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

void Scheduler_Reset(void);
void Scheduler_Tick(void);
uint32_t Scheduler_GetTime(void);

#endif
