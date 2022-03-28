/*
 * scheduler.h
 *
 *  Created on: Mar 27, 2022
 *      Author: ftobler
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "stdint.h"


enum {
	STATE_STOPPED = 0, //task is not started/initialized
	STATE_RUNNING = 1,     //task is currently running
	STATE_WAIT_FLAG = 2,   //task is waiting on flags
	STATE_WAIT_TIME = 3,   //task is waiting time
	STATE_READY = 4,       //task is ready to run
};


typedef struct {
	uint32_t* stackPointer;
	uint32_t timeout;
	uint32_t eventMask;
	uint32_t eventFlags;
	uint8_t state;
} SchedulerTask_t;

typedef void (*SchedulerTaskFunction)();

/**
 * initialized RTOS
 */
void scheduler_init();

/**
 * Add a task to RTOS
 */
void scheduler_addTask(uint32_t id, SchedulerTaskFunction function, uint8_t* stackBuffer, uint32_t stackSize);

/**
 * start RTOS
 */
void scheduler_join();

/**
 * Task sleep function. Call from within a task.
 * Never call it from idle task (0)
 */
void scheduler_task_sleep(uint32_t time);

/**
 * Wait for some event flags
 * If a flag the task is waiting on is set it is awaken
 */
uint32_t scheduler_event_wait(uint32_t eventWaitMask);

/**
 * Set remote tasks event flags
 */
void scheduler_event_set(uint32_t id, uint32_t eventSetMask);

void scheduler_systick_handler();
void scheduler_pendSV_handler();




#endif /* SCHEDULER_H_ */
