#include <stdio.h>
#include <stdlib.h>

#pragma fullpath_file  off 

/* Scheduler includes. */
#include "FreeRTOS.h"

#include "task.h"

#include "CNC2_Main.hpp"

#include "CtrlProcess.hpp"
//#include "HTTP_ServerProcess.hpp"
#include "TELNET_ServerProcess.hpp"
//#include "FTP_ServerProcess.hpp"
#include "RunTimeStats.hpp"
#include "TcpProcess.hpp"

#include "CNC2_axeProcess.hpp"


#include "HeapManager.hpp"

#define configTOTAL_RAMD2_SIZE (30*1024)

uint8_t __attribute((section(".AXI_RAM1"))) ucHeap[ configTOTAL_HEAP_SIZE ] __attribute__ ((aligned (4)));
uint8_t __attribute((section(".DTCM_RAM1"))) ucCCMHeap[ configTOTAL_CCM_HEAP_SIZE ] __attribute__ ((aligned (4)));
uint8_t __attribute((section(".RAM1"))) ucD2Heap[ configTOTAL_RAMD2_SIZE ] __attribute__ ((aligned (4)));
HeapRegion_t xHeapRegions[] =
  {
  { ucHeap ,			configTOTAL_HEAP_SIZE },
/*  { (uint8_t*) SDRAM_START ,	SDRAM_SIZE },  DRAM1 */
  { NULL, 0 }
  };
HeapRegion_t xCCMHeapRegions[] =
  {
  { ucCCMHeap ,			configTOTAL_CCM_HEAP_SIZE },
/*  { (uint8_t*) SDRAM_START ,	SDRAM_SIZE },  DRAM1 */
  { NULL, 0 }
  };
HeapRegion_t xRAMD2HeapRegions[] =
  {
  { ucD2Heap ,			configTOTAL_RAMD2_SIZE },
/*  { (uint8_t*) SDRAM_START ,	SDRAM_SIZE },  DRAM1 */
  { NULL, 0 }
  };

uint16_t baseManagerSizes[] = {32,64,128,340,540,1600,4096,16384};
uint16_t ccmManagerSizes[] = {512,1024,2048,4096,8192,16384};
uint16_t ramD2ManagerSizes[] = {256,1600};

HeapManager_c __attribute((section(".DTCM"))) baseManager(8,baseManagerSizes,0);
HeapManager_c __attribute((section(".DTCM"))) ccmManager(6,ccmManagerSizes,1);
HeapManager_c __attribute((section(".DTCM"))) ramD2Manager(2,ramD2ManagerSizes,2);


#ifdef __cplusplus
 extern "C" {
#endif

void *pvPortMalloc( size_t xWantedSize )
{
  return baseManager.Malloc(xWantedSize,1024);
}

void vPortFree( void *pv )
{
  baseManager.Free(pv);
}

void * pvPortMallocStack( size_t xSize ) 
{
  return ccmManager.Malloc(xSize,1024);
}
void vPortFreeStack( void * pv )
{
  ccmManager.Free(pv);
}

#ifdef __cplusplus
}
#endif


void * operator new(size_t size) { 
  //return (pvPortMalloc(size));
  return baseManager.Malloc(size,0); 
}
void operator delete(void* ptr) {
  //vPortFree(wsk) ;

  uint8_t idx = HeapManager_c::GetManagerId(ptr);
  if(idx == 0)
  {
    baseManager.Free(ptr);
  }
  else if(idx == 1)
  {
    ccmManager.Free(ptr);
  }
  else if(idx == 2)
  {
    ramD2Manager.Free(ptr);
  }

  //baseManager.Free(ptr);
}
 void* operator new[](size_t size) {
  //return (pvPortMalloc(size));
  return baseManager.Malloc(size,0);
}
void operator delete[](void* ptr) {
  //vPortFree(ptr);
  uint8_t idx = HeapManager_c::GetManagerId(ptr);
  if(idx == 0)
  {
    baseManager.Free(ptr);
  }
  else if(idx == 1)
  {
    ccmManager.Free(ptr);
  }
  else if(idx == 2)
  {
    ramD2Manager.Free(ptr);
  }
  //baseManager.Free(ptr);
}

void * AllocFromCCM( size_t xSize ) 
{
  return ccmManager.Malloc(xSize,1024);
}
void FreeToCCM( void * pv ) 
{
  ccmManager.Free(pv);
}


static void prvSetupHeap( void );

  
/*********************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
*/
int ApplMain(void)
{
/*
  SCB->VTOR = 0x00000000;

  //192kB ITCM / 192 kB AXI

  if(FLASH->OPTSR2_CUR != OB_TCM_AXI_SHARED_ITCM192KB)
  {
    HAL_FLASH_OB_Unlock();
    FLASH->OPTSR2_PRG = OB_TCM_AXI_SHARED_ITCM192KB;  192kB ITCM / 192 kB AXI 
    HAL_FLASH_OB_Launch();
    HAL_FLASH_OB_Lock();
  }
*/
  prvSetupHeap();

  #if USE_WATCHDOG == 1
  __HAL_IWDG_RELOAD_COUNTER(&hiwdg1);
  #endif

  new RunTime_c ;

  new CtrlProcess_c(2048,tskIDLE_PRIORITY+4,64,SignalLayer_c::HANDLE_CTRL);
  new TcpProcess_c(2048,tskIDLE_PRIORITY+5,64,SignalLayer_c::HANDLE_TCP);
 // new HttpProcess_c(512,tskIDLE_PRIORITY+3,64,SignalLayer_c::HANDLE_HTTP);
  new TelnetProcess_c(512,tskIDLE_PRIORITY+3,64,SignalLayer_c::HANDLE_TELNET);
 // new FtpProcess_c(1024,tskIDLE_PRIORITY+3,64,SignalLayer_c::HANDLE_FTP);

  new CncAxeProcess_c(2048,tskIDLE_PRIORITY+4,160,SignalLayer_c::HANDLE_AXE);

  vTaskStartScheduler();

  return 0;
}

static void prvSetupHeap( void )
{
    //vPortDefineHeapRegions( xHeapRegions );
  baseManager.DefineHeapRegions(xHeapRegions);

  //vPortDefineHeapRegions( xHeapRegions );
  ccmManager.DefineHeapRegions(xCCMHeapRegions);

  ramD2Manager.DefineHeapRegions(xRAMD2HeapRegions);

  ramD2Manager.allowNullResult = true;
}


#ifdef __cplusplus
 extern "C" {
#endif


void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t __attribute((section(".DTCM"))) xIdleTaskTCB;
static StackType_t __attribute((section(".DTCM"))) uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t __attribute((section(".DTCM"))) xTimerTaskTCB;
static StackType_t __attribute((section(".DTCM"))) uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char *pcTaskName )
{
  printf("stack overflow in %s\n",pcTaskName);
  while(1) {}

}

#ifdef __cplusplus
}
#endif

