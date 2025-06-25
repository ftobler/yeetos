/*
 * app.c
 *
 *  Created on: Jun 8, 2025
 *      Author: ftobler
 */


#include "flortos.h"
#include "stm32_hal.h"
#include "taskmanager.h"
#include "event.h"


extern TIM_HandleTypeDef htim14;

// task entry functions
static void idle_task();
//static void event_queue_task();
static void taskFn1();
static void taskFn2();
static void taskFn3();
static void taskFn4();
static void taskFn5();

#define STACK_SIZE 208

// task stacks
STACK_ATTR static uint8_t stack_idle[STACK_SIZE];
STACK_ATTR static uint8_t stack_event0[STACK_SIZE];
STACK_ATTR static uint8_t stack_event1[STACK_SIZE];
STACK_ATTR static uint8_t stack_event2[STACK_SIZE];
STACK_ATTR static uint8_t stack1[STACK_SIZE];
STACK_ATTR static uint8_t stack2[STACK_SIZE];
STACK_ATTR static uint8_t stack3[STACK_SIZE];
STACK_ATTR static uint8_t stack4[STACK_SIZE];
STACK_ATTR static uint8_t stack5[STACK_SIZE];


// some demo app variables
static int counter0 = 0;
static int counter1 = 0;
static int counterEvtStart = 0;
static int counterEvtTimer = 0;
static int counterEvtUtility = 0;
static int counter2 = 0;
static int counter3 = 0;
static int counter4 = 0;
static int counter5 = 0;
//static int counter3 = 0;
//static int counter4 = 0;
//static int counter5 = 0;


static void startup_fn(void* arg);


void taskmanager_start() {
	scheduler_init();

	scheduler_addTask(0, 0, idle_task, stack_idle, STACK_SIZE);  //idle task
	scheduler_addTask(1, 1, taskFn1, stack1, STACK_SIZE);
	scheduler_addEventHandler(2, 2, stack_event0, STACK_SIZE);
	scheduler_addEventHandler(3, 2, stack_event1, STACK_SIZE);
	scheduler_addEventHandler(4, 2, stack_event2, STACK_SIZE);
//	scheduler_addEventHandler(5, 2, stack_event3, STACK_SIZE);
	scheduler_addTask(5, 3, taskFn2, stack2, STACK_SIZE);
	scheduler_addTask(6, 3, taskFn3, stack3, STACK_SIZE);
	scheduler_addTask(7, 3, taskFn4, stack4, STACK_SIZE);
	scheduler_addTask(8, 4, taskFn5, stack5, STACK_SIZE);  //highest priority task is the last task

	event_push(startup_fn, 0);
	scheduler_join();  // start the RTOS (never exits)
}

static void startup_fn(void* arg) {
	counterEvtStart++;
	HAL_TIM_Base_Start_IT(&htim14);
	event_push(utility_fn, 0);
	event_push(utility_fn, 0);
}

void timer_fn(void* arg) {
	counterEvtTimer++;
//	HAL_TIM_Base_Start_IT(&htim14);
	event_push(startup_fn, 0);
	event_push(utility_fn, 0);

}


void utility_fn(void* arg) {
	counterEvtUtility++;
}





static void idle_task() {
	while (1) {
		counter0++;
		__WFI();  // power saving feature(s) here
	}
}

//static void taskFn1() {
//	while (1) {
//		counter1++;
//		scheduler_task_sleep(8);
//	}
//}

//static void taskFn6() {
//	while (1) {
//		counter6++;
//		scheduler_task_sleep(9);
//	}
//}

//static void taskFn7() {
//	while (1) {
//		counter7++;
//		scheduler_task_sleep(10);
//	}
//}

//static void taskFn8() {
//	while (1) {
//		counter8++;
//		scheduler_task_sleep(11);
//	}
//}


static void taskFn1() {
	while (1) {
		counter1++;
		scheduler_task_sleep(8);
		scheduler_event_set(8, 0x0010);
		uint8_t event = scheduler_event_wait(0x000A);
		scheduler_task_sleep(8);
		if (event & 0x0008)
			scheduler_event_set(7, 0x0004);
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
//	HAL_TIM_Base_Start_IT(&htim14);
	while (1) {
		counter5++;
		scheduler_event_wait(0x0010);
	}
}
