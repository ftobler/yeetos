/*
 * flortos_conf.h
 *
 *  Created on: Jun 8, 2025
 *      Author: ftobler
 */

#ifndef FLORTOS_CONF_H_
#define FLORTOS_CONF_H_

#ifdef __cplusplus
extern "C" {
#endif


#define MAX_NUMBER_OF_TASKS 16
#define EVENT_QUEUE_LENGTH 16

// Define the architecture of the scheduler
//#define SCHEDULER_ARCHITECURE_M0plus
 #define SCHEDULER_ARCHITECURE_M4F


#ifdef __cplusplus
}
#endif


#endif /* FLORTOS_CONF_H_ */
