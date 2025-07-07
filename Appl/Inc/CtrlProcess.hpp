#ifndef CTRL_PROCESS_H
#define CTRL_PROCESS_H

#include "SignalList.hpp"
#include "CommonDef.hpp"

#include "Ethernet.hpp"
//#include "CommandSystem.hpp"
#include "RngClass.hpp"
#include "TimeClass.hpp"
//#include "BootUnit.hpp"


class CtrlProcess_c : public process_c
{
 // CommandSystem_c commandSystem;
  Ethernet_c ethernet;
  
  RngUnit_c rng;
  TimeUnit_c timeUnit;
  //BootUnit_c bootUnit;
 

  public :

  CtrlProcess_c(uint16_t stackSize, uint8_t priority, uint8_t queueSize, HANDLERS_et procId);

  void main(void);

  

};



#endif