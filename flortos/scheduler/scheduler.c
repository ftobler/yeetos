/*
 * scheduler.c
 *
 *  Created on: Mar 27, 2022
 *      Author: ftobler
 */

#include "scheduler.h"
#include "stm32f0xx.h"



static SchedulerTask_t* currentTask;
static SchedulerTask_t* nextTask;


static SchedulerTask_t tasks[8];
static uint32_t numTasks = 0;


void scheduler_init() {
	//set the PendSV interrupt priority to the lowest level 0xF
    *(uint32_t volatile *)0xE000ED20 |= (0xFFU << 16);
}

void scheduler_addTask(SchedulerTaskFunction function, uint8_t priority, uint8_t* stackBuffer, uint32_t stackSize) {
	SchedulerTask_t* task = &tasks[numTasks];
	numTasks++;

	//calculate the stack pointer, the stack grows upside down
	uint32_t* stackPointer = (uint32_t*)((uint32_t)stackBuffer + stackSize);

    //not put data on stack so that if it is popped the task is ready to run
	*(--stackPointer) = 1U << 24; //xPSR (put by ISR)
	*(--stackPointer) = (uint32_t)function; //PC (put by ISR-HW)
	*(--stackPointer) = 0x0000000E; // LR  (put by ISR-HW)
	*(--stackPointer) = 0x0000000C; // R12 (put by ISR-HW)
	*(--stackPointer) = 0x00000003; // R3  (put by ISR-HW)
	*(--stackPointer) = 0x00000002; // R2  (put by ISR-HW)
	*(--stackPointer) = 0x00000001; // R1  (put by ISR-HW)
	*(--stackPointer) = 0x00000000; // R0  (put by ISR-HW)
	*(--stackPointer) = 0x0000000B; // R11  (put by ISR-SW)
	*(--stackPointer) = 0x0000000A; // R10  (put by ISR-SW)
	*(--stackPointer) = 0x00000009; // R9   (put by ISR-SW)
	*(--stackPointer) = 0x00000008; // R8   (put by ISR-SW)
	*(--stackPointer) = 0x00000007; // R7   (put by ISR-SW)
	*(--stackPointer) = 0x00000006; // R6   (put by ISR-SW)
	*(--stackPointer) = 0x00000005; // R5   (put by ISR-SW)
	*(--stackPointer) = 0x00000004; // R4   (put by ISR-SW)

	//put current stack pointer position (including the put data) to the task struct
	task->stackPointer = stackPointer;

	//put the priority
	task->priority = priority;
	task->timeout = 0;
	task->isReady = 1;
	task->isDelayed = 0;

}

void scheduler_join() {
	__disable_irq();
	scheduler_systick_handler();
	__enable_irq();
}


static taskNr = 0;
void scheduler_systick_handler() {
	uwTick++;
	nextTask = &tasks[taskNr];
	taskNr++;
	if (taskNr >= numTasks) {
		taskNr = 0;
	}

	if (currentTask != nextTask) {
		//enable pendSV isr
		*(uint32_t volatile *)0xE000ED04 = (1U << 28);
	}
}

__attribute((naked)) void scheduler_pendSV_handler() {
	__disable_irq();
	register uint32_t* stackPointer asm ("sp");
	register uint32_t register8 asm ("r8");
	register uint32_t register9 asm ("r9");
	register uint32_t register10 asm ("r10");
	register uint32_t register11 asm ("r11");
	if (currentTask != nextTask && currentTask) {
		asm volatile("push {r4-r7}"); //push additional registers
		*(--stackPointer) = register8; //these registers can not be handled by push
		*(--stackPointer) = register9;
		*(--stackPointer) = register10;
		*(--stackPointer) = register11;
		currentTask->stackPointer = stackPointer;
	}

	stackPointer = nextTask->stackPointer;
	*(stackPointer++) = register11; //these registers can not be handled by push
	*(stackPointer++) = register10;
	*(stackPointer++) = register9;
	*(stackPointer++) = register8;
	asm volatile("pop {r4-r7}"); //pop additional registers
	currentTask = nextTask;

    __enable_irq();

    asm volatile("BX lr");  //return
}
