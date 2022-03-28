/*
 * scheduler.h
 *
 *  Created on: Mar 27, 2022
 *      Author: ftobler
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "stdint.h"


typedef struct {
	uint32_t* stackPointer;
	uint8_t priority;
	uint8_t isReady;
	uint8_t isDelayed;
	uint32_t timeout;
} SchedulerTask_t;

typedef void (*SchedulerTaskFunction)();

void scheduler_init();
void scheduler_addTask(SchedulerTaskFunction function, uint8_t priority, uint8_t* stackBuffer, uint32_t stackSize);
void scheduler_join();

void scheduler_systick_handler();
void scheduler_pendSV_handler();



#endif /* SCHEDULER_H_ */
