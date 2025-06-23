/*
 * app.h
 *
 *  Created on: Jun 8, 2025
 *      Author: ftobler
 */

#ifndef TASKMANAGER_H_
#define TASKMANAGER_H_


#ifdef __cplusplus
extern "C" {
#endif


void taskmanager_start();
void timer_fn(void* arg);
void utility_fn(void* arg);


#ifdef __cplusplus
}
#endif


#endif /* TASKMANAGER_H_ */
