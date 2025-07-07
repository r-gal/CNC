
#ifndef GENERAL_CONFIG_H
#define GENERAL_CONFIG_H


#include "main.h"

/* memory management definitions */

#define RUN_FROM_RAM 0
#define RAM_VECTORS_START 0x00000000

#ifdef STM32H725xx
#define MEM_USE_DTCM 1 /* used for processes, stacks and memory managers */
#define MEM_USE_RAM2 1 /* for ethernet */
#define RAM_SECTION_NAME ".AXI_RAM1" 
#define DTCM_SECTION_NAME ".DTCM_RAM1"
#define RAM2_SECTION_NAME ".RAM1"
#endif

#ifdef STM32F446xx
#define MEM_USE_DTCM 0 /* used for processes, stacks and memory managers */
#define MEM_USE_RAM2 0 /* for ethernet */
#define RAM_SECTION_NAME ".RAM1" 
#endif

/* configuration definitions */


#define CONF_USE_OWN_FREERTOS 1

#define CONF_USE_SDCARD 0
#define CONF_USE_RNG 1
#define CONF_USE_BOOTUNIT 0


#define CONF_USE_UART_TERMINAL 0
#define CONF_USE_TIME 1
#define CONF_USE_RUNTIME 1
#define CONF_USE_ETHERNET 1
#define CONF_USE_COMMANDS 1
#define COMMAND_USE_TELNET 1
#define COMMAND_USE_UART 0
#define CONF_USE_WATCHDOG 0
#define CONF_USE_LOGGING 0

#define DEBUG_PROCESS 0

#define USE_FLOAT_VECTORS 1

#define SEGMENT_LIST_LENGTH 512 /* must be 2^n */

#define PULSE_MULTIPLIER 100000
#define F_TIM_MASTER 150000000
#define SPEED_PERIOD 10 /* in ms */

#define TEST_SYNC_TIMER 0 /* assume that sync timer is TIM23 */
#define TEST_SEGLIST 0
#define TEST_AXE 0
#define TEST_AXE_PIPELINE 0
#define TEST_AXE_SPEED 0
#define TEST_AXE_DET 0
#define NO_OF_AXES 4

#define USE_AUTOBASE 0


void * AllocFromCCM( size_t xSize ) ;
void FreeToCCM( void * pv );


#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP 1
/* 0-2 used by RTOS_FAT, 3-4 used by RUNTIMESTATS, 5 used by MyFAT */
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS  6

#define configCHECK_FOR_STACK_OVERFLOW 1

void * AllocFromCCM( size_t xSize ) ;
void FreeToCCM( void * pv );

/* RUNTIME config */
#if CONF_USE_RUNTIME == 1
#define configRUNTIME_THREAD_LOCAL_INDEX 3
#define configRUNTIME_MAX_NO_OF_TASKS 32
#ifdef __cplusplus
 extern "C" {
#endif
uint32_t GetRunTimeTimer(void);
void CreateTaskWrapper(void* task);
void DeleteTaskWrapper(void* task);
#ifdef __cplusplus
}
#endif

#define traceTASK_SWITCHED_IN() {pxCurrentTCB->pvThreadLocalStoragePointers[configRUNTIME_THREAD_LOCAL_INDEX + 0] = (void*)GetRunTimeTimer(); }
#define traceTASK_SWITCHED_OUT() {pxCurrentTCB->pvThreadLocalStoragePointers[configRUNTIME_THREAD_LOCAL_INDEX + 1] = (void*)((uint32_t)pxCurrentTCB->pvThreadLocalStoragePointers[configRUNTIME_THREAD_LOCAL_INDEX + 1] + GetRunTimeTimer() - (uint32_t)pxCurrentTCB->pvThreadLocalStoragePointers[configRUNTIME_THREAD_LOCAL_INDEX + 0]); }
#define traceTASK_CREATE(pxNewTCB) {CreateTaskWrapper((void*)pxNewTCB); pxNewTCB->pvThreadLocalStoragePointers[configRUNTIME_THREAD_LOCAL_INDEX + 0] = 0; pxNewTCB->pvThreadLocalStoragePointers[configRUNTIME_THREAD_LOCAL_INDEX + 1] = 0; }
#define traceTASK_DELETE(pxTCB) {DeleteTaskWrapper((void*)pxTCB); }
#endif

#if CONF_USE_BOOTUNIT == 0
#define VERSION_NAME "v2.1"
#endif

#endif