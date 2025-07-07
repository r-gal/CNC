#ifndef ETHERNETDHCP_H
#define ETHERNETDHCP_H


#include "EthernetCommon.hpp"
#include "EthernetSocket.hpp"
//#include "EthernetIp.hpp"
#include "EthernetUdp.hpp"

#define CNC2_PORT 4010

struct CncFrame_st
{
  uint8_t orderCode;
  uint8_t size;
  uint8_t spare[2];
  int32_t data[32];

};

enum CNC2_orderCode_et
{
  OC_RESET,
  OC_GETSTAT,
  OC_RUN_LINE,
  OC_RUN_ARC,
  OC_RUN_ARC2,
  OC_RUN_DELAY,
  OC_RUN_AUTOBASE,
  OC_RUN_STOP, 
  OC_RUN_PAUSE,
  OC_RUN_PROBE,
  OC_STOP,
  OC_SET_BASE,
  OC_SET_CONFIG,
  OC_SET_SPINDLE_SPEED,
  OC_SET_ZERO,
  OC_PROBE_ACK, /* unused */
  OC_MANUAL_MOVE_START,
  OC_MANUAL_MOVE_STOP,
  OC_RESULT_ACK,
  OC_SUFRACE_OFFSET,
};

class CNC2_socket_c : public SocketUdp_c
{
  int lastReceivedSeqNo;

  uint32_t status;

  

  public:


  void Init(void);

  uint16_t ResponseMessage(CncFrame_st* msg_p,uint8_t oper);

  void HandleLinkStateChange(uint8_t newState);
  void InitStart(void);
  void DeinitStart(void);


  void HandlePacket(uint8_t* packet_p,uint16_t packetSize);

  CNC2_socket_c(void);


};


#endif