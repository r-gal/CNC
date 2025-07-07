 #include <stdio.h>
#include <stdlib.h>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "math.h"


#include "CNC2_Segment.hpp"

Segment_c segmentList[SEGMENT_LIST_LENGTH];

#if USE_SAVE_QUEUES == 1 
QueueHandle_t Segment_c::queue;
QueueHandle_t Segment_c::emptyQueue;
#else
Segment_c* Segment_c::first = nullptr;
Segment_c* Segment_c::last = nullptr;
Segment_c* Segment_c::emptyList = nullptr;
#endif

void Segment_c::InitList(void)
{
  #if USE_SAVE_QUEUES == 1
  queue = xQueueCreate( SEGMENT_LIST_LENGTH, sizeof( void* ) );
  emptyQueue = xQueueCreate( SEGMENT_LIST_LENGTH, sizeof( void* ) );
  #endif

}

void Segment_c::ResetList(void)
{
  #if USE_SAVE_QUEUES == 1
  xQueueReset(queue);
  xQueueReset(emptyQueue);
  #else
  first = nullptr;
  last = nullptr;
  #endif
  for(int i=0;i< SEGMENT_LIST_LENGTH; i++)
  {
    segmentList[i].AddToEmpty();
  }
/*
  int queueSize = uxQueueMessagesWaiting(queue);
  int emptyQueueSize = uxQueueMessagesWaiting(emptyQueue);
  printf("ResetList, Q=%d,EQ=%d\n",queueSize,emptyQueueSize);*/
}


void Segment_c::AddToEmpty(void)
{
  #if USE_SAVE_QUEUES == 1
  Segment_c* segPtr = this;
  xQueueSend(emptyQueue,&segPtr,0);
  #else

  this->next = emptyList;
  emptyList = this;
  #endif
}

void Segment_c::AddToEmptyISR(void)
{
  #if USE_SAVE_QUEUES == 1
  Segment_c* segPtr = this;
 /* int queueSize = uxQueueMessagesWaitingFromISR(queue);
  int emptyQueueSize = uxQueueMessagesWaitingFromISR(emptyQueue);
  printf("AddToEmptyISR, Q=%d,EQ=%d\n",queueSize,emptyQueueSize);*/
  BaseType_t xTaskWokenByReceive = pdFALSE;
  xQueueSendFromISR( emptyQueue, &( segPtr ),&xTaskWokenByReceive );

  #else

  this->next = emptyList;
  emptyList = this;
  #endif
}


Segment_c* Segment_c::GetEmpty(void)
{
  #if USE_SAVE_QUEUES == 1

   /* int queueSize = uxQueueMessagesWaiting(queue);
    int emptyQueueSize = uxQueueMessagesWaiting(emptyQueue);
    printf("GetEmpty, Q=%d,EQ=%d\n",queueSize,emptyQueueSize);*/

    Segment_c* retPtr = nullptr;
    if( xQueueReceive( emptyQueue, &( retPtr ),0 ) != pdPASS )
    {
      retPtr = nullptr;
    }

  #else
  taskENTER_CRITICAL();
  Segment_c* retPtr = emptyList;
  if(emptyList != nullptr)
  {
    if(retPtr->next == nullptr)
    {
      emptyList = nullptr;
    }
    else
    {
      emptyList = retPtr->next;
    }
    retPtr->next = nullptr;
  }
  taskEXIT_CRITICAL();
  #endif
  return retPtr;
}

bool Segment_c::AddToList(void)
{
  #if USE_SAVE_QUEUES == 1

/*      int queueSize2 = uxQueueMessagesWaiting(queue);
    int emptyQueueSize = uxQueueMessagesWaiting(emptyQueue);
    printf("AddToList, Q=%d,EQ=%d\n",queueSize2,emptyQueueSize);*/

  bool needStart = false;
  int queueSize = uxQueueMessagesWaiting(queue);
  if(queueSize == 0)
  {
    needStart = true;
  }
  Segment_c* segPtr = this;
  xQueueSend(queue,&segPtr,0);


  #else
  bool needStart = false;
  taskENTER_CRITICAL();
  this->next = nullptr;
  if(last == nullptr)
  {
    last = this;
    first = this;
    needStart = true;
  }
  else
  {
    last->next = this;
    last = this;
  }
  taskEXIT_CRITICAL();
  #endif
  return needStart;
}
Segment_c* Segment_c::GetFromList(void)
{  
  #if USE_SAVE_QUEUES == 1
/*
        int queueSize = uxQueueMessagesWaiting(queue);
    int emptyQueueSize = uxQueueMessagesWaiting(emptyQueue);
    printf("GetFromList, Q=%d,EQ=%d\n",queueSize,emptyQueueSize);*/

  Segment_c* retPtr = nullptr;
  if( xQueueReceive( queue, &( retPtr ),0 ) != pdPASS )
  {
    retPtr = nullptr;
  }

  #else
  taskENTER_CRITICAL();
  Segment_c* retPtr = first;
  if(retPtr != nullptr)
  {
    if(retPtr->next == nullptr)
    {
      first = nullptr;
      last = nullptr;
    }
    else
    {
      first = retPtr->next;
    }
    retPtr->next = nullptr;
  }
  taskEXIT_CRITICAL();
  #endif
  return retPtr;
}


Segment_c* Segment_c::GetFromListISR(void)
{  
  #if USE_SAVE_QUEUES == 1

 /*       int queueSize = uxQueueMessagesWaitingFromISR(queue);
    int emptyQueueSize = uxQueueMessagesWaitingFromISR(emptyQueue);
    printf("GetFromListISR, Q=%d,EQ=%d\n",queueSize,emptyQueueSize);*/
  Segment_c* retPtr = nullptr;
  BaseType_t xTaskWokenByReceive = pdFALSE;
  if( xQueueReceiveFromISR( queue, &( retPtr ),&xTaskWokenByReceive ) != pdPASS )
  {
    retPtr = nullptr;
  }

  #else
  Segment_c* retPtr = first;
  if(retPtr != nullptr)
  {
    if(retPtr->next == nullptr)
    {
      first = nullptr;
      last = nullptr;
    }
    else
    {
      first = retPtr->next;
    }
    retPtr->next = nullptr;
  }
  #endif
  /*printf("GetFromListISR end\n");*/
  return retPtr;
}


Segment_c* Segment_c::ReadFirstISR(void)
{
  #if USE_SAVE_QUEUES == 1
 /*       int queueSize = uxQueueMessagesWaitingFromISR(queue);
    int emptyQueueSize = uxQueueMessagesWaitingFromISR(emptyQueue);
    printf("ReadFirstISR, Q=%d,EQ=%d\n",queueSize,emptyQueueSize);*/
  Segment_c* retPtr = nullptr;
  if( xQueuePeekFromISR( queue, &( retPtr ) ) != pdPASS )
  {
    retPtr = nullptr;
  }
  return retPtr;

  #else
  return first;
  #endif
}