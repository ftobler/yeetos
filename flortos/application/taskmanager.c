/*
 * app.c
 *
 *  Created on: Jun 8, 2025
 *      Author: ftobler
 */


#include "flortos.h"
#include "stm32_hal.h"
#include "taskmanager.h"


extern TIM_HandleTypeDef htim14;

// task entry functions
static void taskFn0();
static void taskFn1();
static void taskFn2();
static void taskFn3();
static void taskFn4();
static void taskFn5();

// task stacks
STACK_ATTR static uint8_t stack0[128];
STACK_ATTR static uint8_t stack1[128];
STACK_ATTR static uint8_t stack2[128];
STACK_ATTR static uint8_t stack3[128];
STACK_ATTR static uint8_t stack4[128];
STACK_ATTR static uint8_t stack5[128];

// some demo app variables
static int counter0 = 0;
static int counter1 = 0;
static int counter2 = 0;
static int counter3 = 0;
static int counter4 = 0;
static int counter5 = 0;




void taskmanager_start() {
	scheduler_init();
	scheduler_addTask(0, taskFn0, stack0, 128);  //idle task
	scheduler_addTask(1, taskFn1, stack1, 128);
	scheduler_addTask(2, taskFn2, stack2, 128);
	scheduler_addTask(3, taskFn3, stack3, 128);
	scheduler_addTask(4, taskFn4, stack4, 128);
	scheduler_addTask(5, taskFn5, stack5, 128);  //highest priority task is the last task
	scheduler_join();  // start the RTOS (never exits)
}


static void taskFn0() {
	while (1) {
		counter0++;
		__WFI();  // power saving feature(s) here
	}
}


static void taskFn1() {
	while (1) {
		counter1++;
		scheduler_task_sleep(8);
		uint8_t event = scheduler_event_wait(0x000A);
		scheduler_task_sleep(8);
		if (event & 0x0008)
			scheduler_event_set(4, 0x0004);
	}
}


static void taskFn2() {
	while (1) {
		counter2++;
		scheduler_task_sleep(10);
		scheduler_event_set(1, 0x0002);
	}
}


static void taskFn3() {
	while (1) {
		counter3++;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
		scheduler_task_sleep(100);
		scheduler_event_set(1, 0x0008);
	}
}


static void taskFn4() {
	while (1) {
		counter4++;
		scheduler_event_wait_timeout(0x0004, 75);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	}
}


static void taskFn5() {
	HAL_TIM_Base_Start_IT(&htim14);
	while (1) {
		counter5++;
		scheduler_event_wait(0x0010);
	}
}
