/*
 * event.c
 *
 *  Created on: Jun 9, 2025
 *      Author: ftobler
 */


#include "event.h"
#include "stm32_hal.h"
#include "stdbool.h"
#include "flortos.h"


typedef struct {
	YeetEvent function;
	Promise_t promise_to_resolve;
	void* arg;
} Event_t;


static Event_t queue[EVENT_QUEUE_LENGTH];
static int queue_head = 0;
static int queue_tail = 0;
static int queue_size = 0;
static uint64_t promise_id_counter = 1; // 0 means no promise (null) so the first valid id is 1

static Promise_t empty_promise = {
		.id = 0
};


// Check if queue is full
#define queue_full() (queue_size >= EVENT_QUEUE_LENGTH)

// Check if queue is empty
#define queue_empty() (queue_size == 0)


// static local functions
static Event_t* event_pull();

extern volatile SchedulerTask_t* currentTask;
extern uint32_t eventloop_workers_available;


/**
 * Push a new event onto the queue
 */
void event_push(YeetEvent function, void* arg) {
    event_push_promise(function, arg, empty_promise);
}


void event_push_promise(YeetEvent function, void* arg,  Promise_t promise) {
    if (queue_full()) {
        // Queue is full, do not push

    	// ensure the eventloop has a worker serving it
        eventloop_unsuspend();
        return;
    }

    queue[queue_tail].function = function;
    queue[queue_tail].arg = arg;
    queue[queue_tail].promise_to_resolve = promise;

    queue_tail = (queue_tail + 1) % EVENT_QUEUE_LENGTH;
    queue_size++;

	// ensure the eventloop has a worker serving it
    eventloop_unsuspend();
}


/**
 * Gets the next event to process in line. Will not clear it from the queue,
 * but returns a pointer to it.
 */
static Event_t* event_pull() {
    if (queue_empty()) {
        return 0; // Return null pointer on empty queue
    }

    Event_t* event = &queue[queue_head];
    queue_head = (queue_head + 1) % EVENT_QUEUE_LENGTH;
    queue_size--;

    return event;
}


/**
 * worker thread function for the event tasks
 */
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


/**
 * creates a new unresolved promise. The promise is bound to the task that will await it
 */
Promise_t create_promise() {
	Promise_t p;
	p.id = promise_id_counter;
	promise_id_counter++;
	return p;
}


/**
 * Spawns a new task and halts the current task until the new task has finished.
 * Will create and await a promise internally
 */
void await_call(YeetEvent function, void* arg) {
	Promise_t promise = create_promise();
	event_push_promise(function,  arg, promise);
	await_promise(promise);
}







