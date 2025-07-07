 #include <stdio.h>
#include <stdlib.h>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


#include "CtrlProcess.hpp"
#include "RunTimeStats.hpp"

//#include "FileSystem.hpp"


#include "CNC2_socket.hpp"



CtrlProcess_c::CtrlProcess_c(uint16_t stackSize, uint8_t priority, uint8_t queueSize, HANDLERS_et procId) : process_c(stackSize,priority,queueSize,procId,"CONFIG")
{

}

void CtrlProcess_c::main(void)
{
  RunTime_c::Start();
  timeUnit.Init();
  ethernet.Init();

  #if DEBUG_PROCESS > 0
  printf("Ctrl proc started \n");
  #endif

  new CNC2_socket_c;

  while(1)
  {
    releaseSig = true;
    RecSig();
    uint8_t sigNo = recSig_p->GetSigNo();

    

    switch(sigNo)
    {
      default:
      break;

    }
    if(releaseSig)
     {
      delete  recSig_p;
       }
 
  }


}

