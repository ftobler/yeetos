/*
 * app.c
 *
 *  Created on: Jun 8, 2025
 *      Author: ftobler
 */


#include "flortos.h"
#include "stm32_hal.h"
#include "taskmanager.h"
#include "flortos.h"


extern TIM_HandleTypeDef htim11;

// task entry functions
static void idle_task();
//static void event_queue_task();
static void taskFn1();
static void taskFn2();
static void taskFn3();
static void taskFn4();
static void taskFn5();
static void awaited_function(void* arg);
static void promise_resolver_fn(void* arg);
static void promise_event_fn(void* arg);
static void promise_await_fn(void* arg);



#define STACK_SIZE 512

// task stacks
STACK_ATTR static uint8_t stack_idle[STACK_SIZE];
STACK_ATTR static uint8_t stack_event0[STACK_SIZE];
STACK_ATTR static uint8_t stack_event1[STACK_SIZE];
STACK_ATTR static uint8_t stack_event2[STACK_SIZE];
STACK_ATTR static uint8_t stack_event3[STACK_SIZE];
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
static int counter_awaited = 0;
static Promise_t promise1;
static Promise_t promise2;
static int promise_resolved_count = 0;
//static int counter3 = 0;
//static int counter4 = 0;
//static int counter5 = 0;


static void startup_fn(void* arg);


void taskmanager_start() {
	scheduler_init();
	int i = 0;

	scheduler_addTask(i++, 0, idle_task, stack_idle, STACK_SIZE);  //idle task
	scheduler_addTask(i++, 1, taskFn1, stack1, STACK_SIZE);
	scheduler_addEventHandler(i++, 2, stack_event0, STACK_SIZE);
	scheduler_addEventHandler(i++, 2, stack_event1, STACK_SIZE);
	scheduler_addEventHandler(i++, 2, stack_event2, STACK_SIZE);
	scheduler_addEventHandler(i++, 2, stack_event3, STACK_SIZE);
	scheduler_addTask(i++, 3, taskFn2, stack2, STACK_SIZE);
	scheduler_addTask(i++, 3, taskFn3, stack3, STACK_SIZE);
	scheduler_addTask(i++, 3, taskFn4, stack4, STACK_SIZE);
	scheduler_addTask(i++, 4, taskFn5, stack5, STACK_SIZE);  //highest priority task is the last task

//	event_push(startup_fn, 0);
	scheduler_join();  // start the RTOS (never exits)
}

static void startup_fn(void* arg) {
	counterEvtStart++;
	HAL_TIM_Base_Start_IT(&htim11);
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


static void promise_resolver_fn(void* arg) {
    resolve_promise(promise1);
    promise_resolved_count++;
}


static void awaited_function(void* arg) {
    counter_awaited++;
    if ((intptr_t)arg == 0xDEADBEEF) {
        counter_awaited++;
    }
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
		scheduler_flags_set(8, 0x0010);
		uint32_t flags = scheduler_flags_wait(0x000A);
		scheduler_task_sleep(8);
		if (flags & 0x1000) {
			scheduler_event_clear(0x1000);
		}
		if (flags & 0x0008)
			scheduler_flags_set(7, 0x0004);
	}
}


static void taskFn2() {
	while (1) {
		counter2++;
		scheduler_task_sleep(10);
		scheduler_flags_set(1, 0x0002);
		await_call(awaited_function, (void*)0xDEADBEEF);
	}
}


static void taskFn3() {
	while (1) {
		counter3++;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
		scheduler_task_sleep(100);
		scheduler_flags_set(1, 0x0008 | 0x1000);
	}
}


static void promise_event_fn(void* arg) {
	event_push(promise_resolver_fn, 0);
}


static void promise_await_fn(void* arg) {
	promise1 = create_promise();
	event_push(promise_resolver_fn, 0);
	await_promise(promise1);
}


static void taskFn4() {
	while (1) {
		counter4++;
		promise2 = create_promise();
		scheduler_flags_wait_timeout(0x0004, 75);
		resolve_promise(promise2);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

		promise1 = create_promise();
		event_push(promise_event_fn, 0);
		await_promise(promise1);


		event_push(promise_await_fn, 0);
	}
}


static void taskFn5() {
//	HAL_TIM_Base_Start_IT(&htim14);
	while (1) {
		counter5++;
		scheduler_flags_wait(0x0010);
//		await_promise(promise2);
	}
}
