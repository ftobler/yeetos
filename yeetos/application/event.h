/*
 * event.h
 *
 *  Created on: Jun 9, 2025
 *      Author: ftobler
 */

#ifndef EVENT_H_
#define EVENT_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"
#include "flortos_conf.h"

typedef void (*YeetEvent)(void* arg);

typedef struct {
	uint64_t id;
//	bool resolved;
} Promise_t;


void event_push(YeetEvent function, void* arg);
void event_push_promise(YeetEvent function, void* arg, Promise_t promise);

void event_queue_task();

Promise_t create_promise();

void await_call(YeetEvent function, void* arg);



#ifdef __cplusplus
}
#endif


#endif /* EVENT_H_ */
