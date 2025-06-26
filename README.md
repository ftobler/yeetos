# flortos

Preemtive minimalistic RTOS for STM32


## features

* preemtive task scheduling
* cooperative task scheduling
* event queue microtask scheduling
* stackful async await on microtasks
* task priorities
* task delays
* event wait/notify structure with 32 individual flags and optional timeout
* Cortex M0+ and M4F support
* fully static memory allocation


**not supported**
* stop a task
* start a task during runtime
* Any sort of yielding on idle task with ID 0.
* tasks with equal priorities
* No stack overflow/underflow protection


## project integration steps

* copy `flortos.c,` `flortos.h` & `flortos_conf.h` into your existing project.
* create or modify `stm32_hal.h` so it includes your HAL library.
* in your `startup_stm#######.s` file, change the following:
    * replace default `PendSV_Handler` with new `scheduler_pendSV_handler`.
    * replace default `SysTick_Handler` with new `scheduler_systick_handler`.
* on startup initialize the tasks; see `taskmanager_start()` in `taskmanager.c` for an example.


## Minimalist usage example:

```C
static void taskFn0();
static void taskFn1();
STACK_ATTR static uint8_t stack0[128];
STACK_ATTR static uint8_t stack1[128];

void taskmanager_start() {
	scheduler_init();
	scheduler_addTask(0, 0, taskFn0, stack0, 128);  //idle task
	scheduler_addTask(1, 1, taskFn1, stack1, 128);  //highest priority task is the last task
	scheduler_join();  // start the RTOS (never exits)
}

static void taskFn0() {
	while (1) {
		__WFI();  // power saving feature(s) here
	}
}

static void taskFn1() {
	while (1) {
		counter1++;
		scheduler_task_sleep(8);
		uint8_t event = scheduler_flags_wait(0x000A);
		scheduler_task_sleep(8);
		if (event & 0x0008)
			scheduler_event_set(4, 0x0004);
	}
}
```