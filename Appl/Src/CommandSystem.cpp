
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "CommandSystem.hpp"

#if CONF_USE_TIME == 1
#include "TimeClass.hpp"
#endif

#include "HeapManager.hpp"

extern HeapManager_c baseManager;
extern HeapManager_c ccmManager;
extern HeapManager_c ramD2Manager;

extern float batteryVoltage ;
extern float cpuTemperature ;
extern float acuVoltage;
extern float mainVoltage;
#if LON_BOOTUNIT == 1
extern char* versionName;
#else
const char* versionName = VERSION_NAME;
#endif


CommandSystem_c commandSystem;

CommandSystem_c::CommandSystem_c(void)
{



}

#if CONF_USE_TIME == 1
comResp_et Com_gettime::Handle(CommandData_st* comData)
{
  SystemTime_st pxTime;
  TimeUnit_c::GetSystemTime( &pxTime );
  char* strBuf = new char[64];

  TimeUnit_c::PrintTime(strBuf,&pxTime);

  Print(comData->commandHandler,strBuf);

  delete[] strBuf;

  return COMRESP_OK;
}

comResp_et Com_settime::Handle(CommandData_st* comData)
{
  uint32_t hour;
  bool hourValid = FetchParameterValue(comData,"HOUR",&hour,0, 23);

  uint32_t min;
  bool minValid = FetchParameterValue(comData,"MIN",&min,0, 59);

  uint32_t sec;
  bool secValid = FetchParameterValue(comData,"SEC",&sec,0, 59);

  uint32_t day;
  bool dayValid = FetchParameterValue(comData,"DAY",&day,1, 31);

  uint32_t mon;
  bool monValid = FetchParameterValue(comData,"MON",&mon,1, 12);

  uint32_t year;
  bool yearValid = FetchParameterValue(comData,"YEAR",&year,0, 2099);

  SystemTime_st pxTime;

  printf("COM = %s\n",comData->buffer);
  for(int i =0;i< comData->argsCnt;i++)
  {
    printf("ARG_%d %s = %s\n",i,comData->buffer+comData->argStringPos[i],comData->buffer+comData->argValPos[i]);
 

  }
  if(hourValid || minValid || secValid || dayValid || monValid || yearValid)
  {
    
    TimeUnit_c::GetSystemTime( &pxTime );

    if(hourValid) { pxTime.Hour = hour; }
    if(minValid) { pxTime.Minute = min; }
    if(secValid) { pxTime.Second = sec; }
    if(dayValid) { pxTime.Day = day; }
    if(monValid) { pxTime.Month = mon; }
    if(yearValid) { pxTime.Year = year; }

    TimeUnit_c::SetSystemTime( &pxTime );
  }


  char* strBuf = new char[64];

  TimeUnit_c::GetSystemTime( &pxTime );

  TimeUnit_c::PrintTime(strBuf,&pxTime);

  Print(comData->commandHandler,strBuf);

  delete[] strBuf;

  return COMRESP_OK;

}
#endif

comResp_et Com_meminfo::Handle(CommandData_st* comData)
{
  HeapManager_c* manager;
  int actionIdx = FetchParameterString(comData,"RAM");

  if(actionIdx>=0)
  {
    if(strcmp(comData->buffer+comData->argValPos[actionIdx],"SRAM") == 0)
    {
      manager = & baseManager;
    }
    #if MEM_USE_DTCM == 1
    else if (strcmp(comData->buffer+comData->argValPos[actionIdx],"DTCM") == 0)
    { 
      manager = & ccmManager;
    }
    #endif
    #if MEM_USE_RAM2 == 1
    else if(strcmp(comData->buffer+comData->argValPos[actionIdx],"RAMD2") == 0)
    {
      manager = & ramD2Manager;
    }
    #endif
    else
    {
      return COMRESP_VALUE;
    }
  } 
  else
  {
      manager = & baseManager;
  }


  uint16_t noOfAlloc, noOfFree;
  uint32_t noOfFreeMem, noOfFreePrim, noOfFreeMin;
  uint16_t noOfReq;
  uint16_t noOfRel;
  uint16_t size;
  uint8_t idx=0;

  char* strBuf = new char[64];

  do
  {
    //xPortGetHeapListStats(&noOfAlloc,&noOfFree,&size,idx);
    manager->GetHeapListStats(&noOfAlloc,&noOfFree,&size,idx);

    if(size > 0 )
    {
      sprintf(strBuf,"MEM SIZE=%d, ALLOC=%d, FREE=%d\n",size,noOfAlloc,noOfFree);
      Print(comData->commandHandler,strBuf);

    }
    idx++;
  }while(size > 0);
/*
  noOfFreeMin =  xPortGetMinimumEverFreeHeapSize( );
  noOfFree = xPortGetUnallocatedHeapSize( );
  noOfFreePrim = xPortGetPrimaryUnallocatedHeapSize( );
*/
  noOfFreeMin =  manager->GetMinimumEverFreeHeapSize( );
  noOfFreeMem = manager->GetUnallocatedHeapSize( );
  noOfFreePrim = manager->GetPrimaryUnallocatedHeapSize( );
  
  sprintf(strBuf,"NIM_FREE=%d, FREE=%d, FREE_PRIM=%d\n",noOfFreeMin,noOfFreeMem,noOfFreePrim);
  Print(comData->commandHandler,strBuf);

  #if STORE_EXTRA_MEMORY_STATS == 1
  idx = 0;
  do
  {
    //xPortGetHeapLargestRequests(&noOfReq,&noOfAlloc,&size,idx);
    manager->GetHeapLargestRequests(&noOfReq,&noOfRel,&noOfAlloc,&size,idx);
    if(size > 0 )
    {
      sprintf(strBuf,"MEM FREQ SIZE=%d, REQ=%d, REL=%d, NEW=%d\n",size,noOfReq,noOfRel,noOfAlloc);
      Print(comData->commandHandler,strBuf);

    }
    idx++;
  }while(size > 0);
  #endif

  delete[] strBuf;

  return COMRESP_OK;

}


comResp_et Com_memdetinfo::Handle(CommandData_st* comData)
{
  HeapManager_c* manager;
  int actionIdx = FetchParameterString(comData,"RAM");

  if(actionIdx>=0)
  {
    if(strcmp(comData->buffer+comData->argValPos[actionIdx],"SRAM") == 0)
    {
      manager = & baseManager;
    }
    #if MEM_USE_DTCM == 1
    else if (strcmp(comData->buffer+comData->argValPos[actionIdx],"DTCM") == 0)
    { 
      manager = & ccmManager;
    }
    #endif
    #if MEM_USE_RAM2 == 1
    else if(strcmp(comData->buffer+comData->argValPos[actionIdx],"RAMD2") == 0)
    {
      manager = & ramD2Manager;
    }
    #endif
    else
    {
      return COMRESP_VALUE;
    }
  } 
  else
  {
      manager = & baseManager;
  }

  uint32_t min;
  bool minValid = FetchParameterValue(comData,"MIN",&min,0, 0xFFFF);

  uint32_t max;
  bool maxValid = FetchParameterValue(comData,"MAX",&max,0, 0xFFFF);

  uint32_t idx;
  bool idxValid = FetchParameterValue(comData,"IDX",&idx,0, 0xFFFF);

  if(minValid & maxValid & idxValid == false)
  {
    return COMRESP_NOPARAM;
  }

  void* helpPtr = nullptr;
  uint16_t size;
  uint8_t state;
  UBaseType_t allocTaskNumber;
  int id;

  char* strBuf = new char[128];

  UBaseType_t arraySize = uxTaskGetNumberOfTasks();
  TaskStatus_t* pxTaskStatusArray = new TaskStatus_t[arraySize];
  arraySize = uxTaskGetSystemState( pxTaskStatusArray, arraySize, NULL );

  do
  {
    
    manager->GetHeapAllocationDetails(&helpPtr,&size,&allocTaskNumber,&state,&id,idx,min,max);

    if(helpPtr != nullptr)
    {

      const char* taskName = "del";
      if(allocTaskNumber != 0xFFFFFFFF)
      {
        for(int i=0;i<arraySize;i++)
        {
          if(allocTaskNumber == pxTaskStatusArray[i].xTaskNumber)
          {
            taskName = pxTaskStatusArray[i].pcTaskName;
            break;
          }
        }
      }
      else
      {
        taskName = "No task";
      }

      sprintf(strBuf,"size=%d,state=%d,id=%d,taskNo=%d,task=%s\n",size,state,id,allocTaskNumber, taskName );
      Print(comData->commandHandler,strBuf);


    }


  } while(helpPtr != nullptr);
  delete[] strBuf;
  delete[] pxTaskStatusArray;




  return COMRESP_OK;
}

comResp_et Com_gettasks::Handle(CommandData_st* comData)
{
  int noOfTasks = uxTaskGetNumberOfTasks();
  char* printBuffer = new char[40*noOfTasks];  
  vTaskList(printBuffer);
  Print(comData->commandHandler,printBuffer);
  delete[] printBuffer;
  return COMRESP_OK;
}



comResp_et Com_sysinfo::Handle(CommandData_st* comData)
{
  char* strBuf = new char[128];
  #if CONF_USE_TIME == 1
  SystemTime_st* restartTime;
  restartTime = TimeUnit_c::GetRestartTime();
  #else
  strcpy(strBuf,"UNN");
  #endif

  sprintf(strBuf,"SYSTEM INFO:\n\n Last restart:\n");
  Print(comData->commandHandler,strBuf);

  #if CONF_USE_TIME == 1
  TimeUnit_c::PrintTime(strBuf,restartTime);  
  Print(comData->commandHandler,strBuf);
  #endif

  sprintf(strBuf,"Version: %s\n",versionName);
  Print(comData->commandHandler,strBuf);
  
  #if LON_USE_PWR_MGMT == 1
  sprintf(strBuf,"V BAT           = %.2fV\n",batteryVoltage);
  Print(comData->commandHandler,strBuf);

  sprintf(strBuf,"V ACU           = %.2fV\n",acuVoltage);
  Print(comData->commandHandler,strBuf);

  sprintf(strBuf,"V MAIN          = %.2fV\n",mainVoltage);
  Print(comData->commandHandler,strBuf);

  sprintf(strBuf,"CPU temperature = %.2fC\n",cpuTemperature);
  Print(comData->commandHandler,strBuf);
  #endif
  Print(comData->commandHandler,"\nEND.\n");



  delete[] strBuf;
  return COMRESP_OK;
}

comResp_et Com_siginfo::Handle(CommandData_st* comData)
{
  char* strBuf = new char[128];

  for(int handler = 0; handler < SignalList_c::HANDLE_NO_OF; handler++)
  {
    QueueHandle_t queueHandler = SignalLayer_c::GetHandler((SignalList_c::HANDLERS_et)handler);

    if(queueHandler != NULL)
    {
      uint16_t actFree = uxQueueSpacesAvailable(queueHandler);
      uint16_t actUsage = uxQueueMessagesWaiting(queueHandler);


      sprintf(strBuf,"Handler=%2d actFree=%4d actUsage=%4d maxUsage=%4d rejected=%4d sent =%d\n",
      handler,
      actFree,
      actUsage,
      SignalLayer_c::signalStatistics[handler].maxQueueUsage,
      SignalLayer_c::signalStatistics[handler].noOfRejectedSignals,
      SignalLayer_c::signalStatistics[handler].noOfSendSignals);

      Print(comData->commandHandler,strBuf);
    }

  }

  delete[] strBuf;
  return COMRESP_OK;
}

/*
void CommandSystem_c::HandleDINFP(void)
{
  UsartSendText_c* sig_p = new UsartSendText_c;

  sprintf(sig_p->text,"Last restart: %02d-%02d-%04d %02d:%02d:%02d.\n Cause=%02X\n",
  restartDate.RTC_Date,
  restartDate.RTC_Month,
  restartDate.RTC_Year+2000,
  restartTime.RTC_Hours,
  restartTime.RTC_Minutes,
  restartTime.RTC_Seconds,
  restartCause); 

  sig_p->Send();

  DebugData_c* debugData_p = DebugData_c::ownPtr;

  sig_p = new UsartSendText_c;
  debugData_p->PrintCommonData(sig_p->text);
  sig_p->Send();

    sig_p = new UsartSendText_c;
  debugData_p->PrintCommonData2(sig_p->text);
  sig_p->Send();

  
  sig_p = new UsartSendText_c;
  debugData_p->PrintFaultData(sig_p->text);
  sig_p->Send();

  sig_p = new UsartSendText_c;
  debugData_p->PrintFaultData2(sig_p->text);
  sig_p->Send();

  for(int i=0;i<NO_OF_TASKS; i++)
  {
    sig_p = new UsartSendText_c;

    debugData_p->debugTaskData[i].PrintTaskData(sig_p->text,debugData_p->totalRunTime);

    sig_p->Send();

    sig_p = new UsartSendText_c;

    debugData_p->debugTaskData[i].PrintTaskHistory(sig_p->text);

    sig_p->Send();


    
  }

void CommandHandler_c::SetRestartRime(void)
{
  RTC_GetTime(RTC_Format_BIN, &restartTime);
  RTC_GetDate(RTC_Format_BIN, &restartDate);

  restartCause = RCC->CSR>>24;
  RCC_ClearFlag();
}

}*/