/*
 * scheduler.c
 *
 *  Created on: Mar 27, 2022
 *      Author: ftobler
 */

#include "flortos.h"
#include "stm32_hal.h"
#include "flortos_conf.h"
#include "stdbool.h"

// --- TypeDefs ---

typedef struct {
	YeetEvent function;
	Promise_t promise_to_resolve;
	void* arg;
} Event_t;


// --- Private Variables ---

static Event_t queue[EVENT_QUEUE_LENGTH];
static int queue_head = 0;
static int queue_tail = 0;
static int queue_size = 0;
static uint64_t promise_id_counter = 1; // 0 means no promise (null) so the first valid id is 1

static const Promise_t empty_promise = { .id = 0 };

volatile SchedulerTask_t* current_task = 0;
static volatile SchedulerTask_t* next_task;

static SchedulerTask_t tasks[MAX_NUMBER_OF_TASKS] = {0};
static uint32_t highest_task = 0;
static uint32_t highest_event_task = 0;
volatile uint32_t eventloop_workers_available = 1;  // if 0 during startup a worker is woken up on event adding. prevent that.

// --- Private Function Prototypes ---

static Event_t* event_pull();
static void event_push_promise(YeetEvent function, void* arg,  Promise_t promise);
static void eventloop_suspend();
static void eventloop_unsuspend();
static SchedulerTask_t* scheduler_addTask_internal(uint32_t id, uint32_t preemptive_group, SchedulerTaskFunction function, uint8_t* stackBuffer, uint32_t stackSize);
static void scheduler_work();
static void scheduler_task_time_update();
static void eventloop_unsuspend_internal();

// --- Macros ---

#define queue_full() (queue_size >= EVENT_QUEUE_LENGTH)
#define queue_empty() (queue_size == 0)
#define pendsv_set_lowest_priority() (*(uint32_t volatile *)0xE000ED20 |= (0xFFU << 16))  // set the PendSV interrupt priority to the lowest level 0xF
#define pendsv_enable_isr() (*(uint32_t volatile *)0xE000ED04 = (1U << 28))  // enable PendSV interrupt

// --- Public Functions ---

void event_push(YeetEvent function, void* arg) {
    event_push_promise(function, arg, empty_promise);
}


void event_queue_task() {
	while (1) {
		Event_t* event = event_pull();
		if (event) {
			// run the task body function of this event task
			event->function(event->arg);

			// in case the current event had a promise it would resolve
			// after it runs out, resolve it here
			resolve_promise(event->promise_to_resolve);

		} else {
			// there is no work. go sleeping.
			eventloop_suspend();
		}
	}
}


Promise_t create_promise() {
	Promise_t p;
	p.id = promise_id_counter;
	promise_id_counter++;
	return p;
}


void await_call(YeetEvent function, void* arg) {
	Promise_t promise = create_promise();
	event_push_promise(function,  arg, promise);
	await_promise(promise);
}


void scheduler_init() {
    pendsv_set_lowest_priority();
}


void scheduler_join() {
	__disable_irq();
	eventloop_unsuspend_internal();  // force unsuspend event task
	eventloop_workers_available = 1;
	scheduler_work();
	__enable_irq();
}


void scheduler_task_sleep(uint32_t time) {
	volatile SchedulerTask_t* task = current_task;
	task->timeout = time;
	task->state = STATE_WAIT_TIME;
	scheduler_work();
}


uint32_t scheduler_flags_wait(uint32_t eventWaitMask) {
	volatile SchedulerTask_t* task = current_task;
	task->eventMask = eventWaitMask;
	task->state = STATE_WAIT_FLAG;
	scheduler_work();
	uint32_t events = task->eventFlags;
	task->eventFlags &= ~eventWaitMask;  // clear the flags the task was waiting for
	return events;
}


uint32_t scheduler_flags_wait_timeout(uint32_t eventWaitMask, uint32_t time) {
	volatile SchedulerTask_t* task = current_task;
	task->eventMask = eventWaitMask;
	task->timeout = time;
	task->state = STATE_WAIT_FLAG;
	scheduler_work();
	uint32_t events = task->eventFlags;
	task->eventFlags &= ~eventWaitMask;  // clear the flags the task was waiting for
	return events;
}


void scheduler_flags_set(uint32_t id, uint32_t eventSetMask) {
	// set remote tasks flags
	tasks[id].eventFlags |= eventSetMask;
    scheduler_work();
}


void scheduler_event_clear(uint32_t eventMask) {
	volatile SchedulerTask_t* task = current_task;
	task->eventFlags &= ~eventMask;
}


void await_promise(Promise_t promise) {
	volatile SchedulerTask_t* task = current_task;
	task->promise_id = promise.id;
	task->state = STATE_WAIT_PROMISE;
	// mark as suspended since this is the current worker, but this task
	// needs now to wait.
	eventloop_workers_available = 0;

	// ensure the eventloop has a worker serving it
	eventloop_unsuspend();  // will switch tasks
}


void resolve_promise(Promise_t promise) {
    //TODO: must interact with scheduler to pull some task out of suspension
	uint32_t id = highest_task;
	SchedulerTask_t* task = &tasks[id];
	// go through every task id, starting from the highest priority task
	while (id) {
		if (task->state == STATE_WAIT_PROMISE) {
			if (task->promise_id == promise.id) {
				// promise is hereby resolved
				task->promise_id = 0;
				task->state = STATE_READY;
				break; // nothing further to do here. Promise is resolved. We do not switch tasks
			}
		}
		// loop variables
		id--;
		task--;
	}
}


void scheduler_systick_handler() {
	uwTick++;
	scheduler_task_time_update();
	//scheduler_work();
}


void scheduler_addEventHandler(uint32_t id, uint32_t preemptive_group, uint8_t* stackBuffer, uint32_t stackSize) {
	SchedulerTask_t* task = scheduler_addTask_internal(id, preemptive_group, event_queue_task, stackBuffer, stackSize);
	task->state = STATE_WAIT_EVENTLOOP;  // event tasks start suspended
	task->is_event_task = 1;
}


void scheduler_addTask(uint32_t id, uint32_t preemptive_group, SchedulerTaskFunction function, uint8_t* stackBuffer, uint32_t stackSize) {
	scheduler_addTask_internal(id, preemptive_group, function, stackBuffer, stackSize);
}

// --- Private Functions ---

static Event_t* event_pull() {
    if (queue_empty()) {
        return 0; // Return null pointer on empty queue
    }

    Event_t* event = &queue[queue_head];
    queue_head = (queue_head + 1) % EVENT_QUEUE_LENGTH;
    queue_size--;

    return event;
}


static void event_push_promise(YeetEvent function, void* arg,  Promise_t promise) {
    if (!queue_full()) {
    	// push
        queue[queue_tail].function = function;
        queue[queue_tail].arg = arg;
        queue[queue_tail].promise_to_resolve = promise;

        queue_tail = (queue_tail + 1) % EVENT_QUEUE_LENGTH;
        queue_size++;
    }
	// ensure the eventloop has a worker serving it
    eventloop_unsuspend();
}


static void eventloop_suspend() {
	volatile SchedulerTask_t* task = current_task;
	task->state = STATE_WAIT_EVENTLOOP;
	eventloop_workers_available = 0;  // mark as suspended, so eventloop gets re-started
	scheduler_work();  // will switch tasks
}


static SchedulerTask_t* scheduler_addTask_internal(uint32_t id, uint32_t preemptive_group, SchedulerTaskFunction function, uint8_t* stackBuffer, uint32_t stackSize) {
	SchedulerTask_t* task = &tasks[id];
	if (id > highest_task) {
		highest_task = id;
	}
	if (id > highest_event_task && function==event_queue_task) {
		highest_event_task = id;
	}

	// task id must be in range
	if (id >= MAX_NUMBER_OF_TASKS) {
		while (1) {
			//trap. Make sure task id is in range. Configure more tasks in _conf.h
		}
	}

	// stack must be 4 byte aligned or ARM will not be happy. Not aligning may impact speed, or crash the core.
	if ((uint32_t)stackBuffer & 0x03) {
		while (1) {
			//trap. Make sure stack is 4 byte aligned.
		}
	}

	// calculate the stack pointer, the stack grows upside down
	uint32_t* stackPointer = (uint32_t*)((uint32_t)stackBuffer + stackSize);

	#ifdef SCHEDULER_ARCHITECURE_M0plus

    // not put data on stack so that if it is popped the task is ready to run
	*(--stackPointer) = 1U << 24; // xPSR (put by ISR)
	*(--stackPointer) = (uint32_t)function; // PC (put by ISR-HW)
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

	#endif
	#ifdef SCHEDULER_ARCHITECURE_M4F

    //not put data on stack so that if it is popped the task is ready to run
	*(--stackPointer) = 1U << 24; // xPSR (put by ISR)
	*(--stackPointer) = (uint32_t)function; // PC (put by ISR-HW)
	*(--stackPointer) = 0x0000000E; // LR  (put by ISR-HW) //also r14
	*(--stackPointer) = 0x0000000C; // R12 (put by ISR-HW)
	*(--stackPointer) = 0x00000003; // R3  (put by ISR-HW)
	*(--stackPointer) = 0x00000002; // R2  (put by ISR-HW)
	*(--stackPointer) = 0x00000001; // R1  (put by ISR-HW)
	*(--stackPointer) = 0x00000000; // R0  (put by ISR-HW)

	for (int i = 16;i < 32; i++) {
		*(--stackPointer) = 0; // FPU register   (put by ISR-SW)
	}

	*(--stackPointer) = 0x00000007; // R7   (put by ISR-SW)
	*(--stackPointer) = 0x00000006; // R6   (put by ISR-SW)
	*(--stackPointer) = 0x00000005; // R5   (put by ISR-SW)
	*(--stackPointer) = 0x00000004; // R4   (put by ISR-SW)
	*(--stackPointer) = 0x00000008; // R8   (put by ISR-SW)
	*(--stackPointer) = 0x00000009; // R9   (put by ISR-SW)
	*(--stackPointer) = 0x0000000A; // R10  (put by ISR-SW)
	*(--stackPointer) = 0x0000000B; // R11  (put by ISR-SW)

	#endif

	// not put data on stack so that if it is popped the task is ready to run

	// put current stack pointer position (including the put data) to the task struct
	task->stackPointer = stackPointer;

	// put the priority
	task->timeout = 0;
	task->eventFlags = 0;
	task->eventMask = 0;
	task->state = STATE_READY;
	task->promise_id = 0;
	task->preemptive_group = preemptive_group;
	task->is_event_task = 0;

	return task;
}


static void eventloop_unsuspend() {
	//ensure the eventloop has a worker serving it, this one is going to suspend
	if (current_task == 0) {
		return;
	}
	if (eventloop_workers_available == 0) {
		eventloop_unsuspend_internal();
		eventloop_workers_available = 1;
	}
	scheduler_work();  // will switch tasks
}


__attribute__((optimize("O0")))
static void eventloop_unsuspend_internal() {
    uint32_t id = highest_event_task;
	SchedulerTask_t* task = &tasks[id];
	// go through every task id, starting from the highest priority task
	while (id) {
		if (task->is_event_task) {
			if (task->state == STATE_WAIT_EVENTLOOP || task->state == STATE_READY) {
				task->state = STATE_READY;
				return;
			}
		}
		// loop variables
		id--;
		task--;
	}
	while (1) {
		// FATAL ERROR
		// No task available to serve event loop.
		// All tasks are waiting for promises. Or are otherwise blocked
		// Ensure there are
		//    a.) enough tasks available to serve the application,
		//    b.) no logic exhausts tasks by perpetually blocking,
		//    c.) avoid eventloop tasks being blocked by other events
	}
}


__attribute__((optimize("O0")))
static void scheduler_work() {
	uint32_t id = highest_task;
	SchedulerTask_t* task = &tasks[id];
	// go through every task id, starting from the highest priority task
	while (id) {
		// update task
		if (task->state == STATE_WAIT_FLAG) {
			// check if at least one flag which is masked is set
			if (task->eventFlags & task->eventMask) {
				// task is ready to run
				task->state = STATE_READY;
			}
		}
		// if task is runnable then run it.
		if (task->state == STATE_READY) {
			// found task to run. Exit loop.
			next_task = task;
			break;
		}
		// loop variables
		id--;
		task--;
	}
	if (id == 0) {
		// when nothing else to do run idle task.
		// since loop has gotten to id=0 the idle task is already on the pointer.
		next_task = task;
	}

	// check if the new task has a different preemptive group. otherwise exit.
	if (current_task &&
		next_task->preemptive_group == current_task->preemptive_group &&
		current_task->state == STATE_RUNNING) {
		// do not switch tasks if the preemptive group is the same
		next_task = current_task;
	}

	// switch task if needed
	if (current_task != next_task) {
		// enable pendSV isr
		pendsv_enable_isr();
	}
}


static void scheduler_task_time_update() {
	uint32_t id = highest_task;
	SchedulerTask_t* task = &tasks[id];
	// go through every task id, starting from the highest priority task
	while (id) {
		// update task
		if (task->timeout) {
			if (task->timeout == 1) {
				task->timeout = 0;
				task->state = STATE_READY;
			} else {
				task->timeout--;
			}
		}

		// loop variables
		id--;
		task--;
	}
}


// --- Interrupt Handlers ---

__attribute((naked))
__attribute__((optimize("O0")))
void scheduler_pendSV_handler() {

	#ifdef SCHEDULER_ARCHITECURE_M0plus

	__disable_irq();
	volatile register uint32_t* stackPointer asm ("sp");

	if (current_task) {
		asm volatile("push {r4-r7}"); // push additional registers
		asm volatile("mov r3, r8  \n push {r3}" : : : "r3","memory"); // these registers can not be handled by push
		asm volatile("mov r3, r9  \n push {r3}" : : : "r3","memory"); // "r3" in the clobber list informs the compiler that r3 will be used in this section
		asm volatile("mov r3, r10 \n push {r3}" : : : "r3","memory"); // "memory" in the clobber list informs the compiler that memory content may have changed
		asm volatile("mov r3, r11 \n push {r3}" : : : "r3","memory"); // "sp" in clobber list is deprecated and not needed
		current_task->stackPointer = (uint32_t*)stackPointer;
	}

	stackPointer = next_task->stackPointer;
	asm volatile("pop {r3}\n mov r11, r3" : : : "r3","memory");// these registers can not be handled by push
	asm volatile("pop {r3}\n mov r10, r3" : : : "r3","memory");
	asm volatile("pop {r3}\n mov  r9, r3" : : : "r3","memory");
	asm volatile("pop {r3}\n mov  r8, r3" : : : "r3","memory");
	asm volatile("pop {r4-r7}"); // pop additional registers
	current_task = next_task;

    __enable_irq();
    asm volatile("BX lr");  // return

	#endif

	#ifdef SCHEDULER_ARCHITECURE_M4F

		__disable_irq();
	volatile register uint32_t* stackPointer asm ("sp");
	asm volatile("isb"); // Flush instruction pipeline

	if (currentTask) {
		// Save FPU state (floating-point registers S16-S31)
		asm volatile("vstmdb sp!, {s16-s31}" : : : "memory"); // Store FPU registers
		// Save general-purpose registers R4-R11
		asm volatile("push {r4-r11}");
		// Memory barriers to ensure everything is written before switching
		asm volatile("dsb \n isb"); // Ensures memory accesses and pipeline flushing
		// Save stack pointer of the current task
		currentTask->stackPointer = (uint32_t*)stackPointer;
	}

	// Switch to the new task's stack pointer
	stackPointer = nextTask->stackPointer;

	// Ensure the stack is properly aligned (optional, depending on alignment settings)
	asm volatile("dsb \n isb"); // Synchronization to make sure all changes are applied
	// Restore general-purpose registers R4-R11
	asm volatile("pop {r4-r11}");
	// Restore FPU state (floating-point registers S16-S31)
	asm volatile("vldmia sp!, {s16-s31}" : : : "memory");

	// Update the current task to the next task
	currentTask = nextTask;

	__enable_irq();

	// Make sure no pending instructions exist before returning
	asm volatile("dsb \n isb"); // Synchronize memory and instruction pipelines before returning
	asm volatile("BX lr");  // Return from handler

	#endif
}

