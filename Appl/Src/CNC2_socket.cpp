#include <stdio.h>
#include <stdlib.h>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


#include "CNC2_socket.hpp"
#include "SignalList.hpp"
#include "EthernetBuffers.hpp"

#include "CNC2_axeProcess.hpp"

CNC2_socket_c::CNC2_socket_c(void)  : SocketUdp_c(CNC2_PORT)
{
  status = 0;
  
}

void CNC2_socket_c::Init(void)
{






}

void CNC2_socket_c::HandleLinkStateChange(uint8_t newState)
{

}

void CNC2_socket_c::InitStart(void)
{

}

void CNC2_socket_c::DeinitStart(void)
{

}



uint16_t  CNC2_socket_c::ResponseMessage(CncFrame_st* frame_p,uint8_t oper)
{
  switch(frame_p->orderCode)
  {
    case OC_RESET:
      {
        frame_p->size = 0;
        lastReceivedSeqNo = -1;

        CNC_ResetSig_c* sig_p = new CNC_ResetSig_c;
        sig_p->Send();
      }
      return 4;
    case OC_GETSTAT:
      frame_p->size = 8;

      for(int i =0;i<NO_OF_AXES;i++)
      {
        frame_p->data[i] = CncAxeProcess_c::pos[i]  ; 
      }
      frame_p->data[5] = CncAxeProcess_c::actSeqNo;
      frame_p->data[6] = CncStatus_c::GetStatus();
      frame_p->data[7] = lastReceivedSeqNo ;
      return 36;
    case OC_RUN_LINE:
      {

        CNC_moveSig_c* sig_p = new CNC_moveSig_c;
        sig_p->seqNo = frame_p->data[0];
        lastReceivedSeqNo = frame_p->data[0];

        sig_p->speedStart = ((float)frame_p->data[2])/1000 ;
        sig_p->speedEnd = ((float)frame_p->data[3])/1000 ;
        sig_p->maxSpeed = ((float)frame_p->data[4])/1000 ;
        sig_p->maxAcceleration = ((float)frame_p->data[5])/1000 ;
      
        sig_p->x = frame_p->data[6];
        sig_p->y = frame_p->data[7];
        sig_p->z = frame_p->data[8];
        sig_p->a = frame_p->data[9];

        sig_p->ignoreLimiters = (frame_p->data[10] != 0);

        sig_p->moveType = MOVE_LINE;
        #if TEST_AXE_PIPELINE == 1
        printf("rec line, seq=%d\n",sig_p->seqNo);
        #endif
        sig_p->Send();

        frame_p->data[0] = CncAxeProcess_c::actSeqNo;
        frame_p->data[1] = lastReceivedSeqNo ;
        frame_p->size = 3;
      }
      return 8;
    case OC_RUN_ARC:
      {
        CNC_moveSig_c* sig_p = new CNC_moveSig_c;        
        lastReceivedSeqNo = frame_p->data[0];

        sig_p->speedStart = ((float)frame_p->data[2])/1000 ;
        sig_p->speedEnd = ((float)frame_p->data[3])/1000 ;
        sig_p->maxSpeed = ((float)frame_p->data[4])/1000 ;
        sig_p->maxAcceleration = ((float)frame_p->data[5])/1000 ;
      
        sig_p->x = frame_p->data[6];
        sig_p->y = frame_p->data[7];
        sig_p->z = frame_p->data[8];
        sig_p->a = frame_p->data[9];

        sig_p->cx = frame_p->data[10];
        sig_p->cy = frame_p->data[11];
        sig_p->cz = frame_p->data[12];

        sig_p->clockwise = (frame_p->data[16] == 1);
        sig_p->plane = (PLANE_SELECT_et)frame_p->data[17];
        sig_p->turns = frame_p->data[18];

        sig_p->moveType = MOVE_ARC;
        #if TEST_AXE_PIPELINE == 1
        printf("rec arc, seq=%d\n",sig_p->seqNo);
        #endif

        sig_p->Send();

        frame_p->data[0] = CncAxeProcess_c::actSeqNo;
        frame_p->data[1] = lastReceivedSeqNo ;
        frame_p->size = 3;
      }
      return 8;
    case OC_RUN_ARC2:
      {
        CNC_moveSig_c* sig_p = new CNC_moveSig_c;
        sig_p->seqNo = frame_p->data[0];
        lastReceivedSeqNo = frame_p->data[0];

        sig_p->speedStart = ((float)frame_p->data[2])/1000 ;
        sig_p->speedEnd = ((float)frame_p->data[3])/1000 ;
        sig_p->maxSpeed = ((float)frame_p->data[4])/1000 ;
        sig_p->maxAcceleration = ((float)frame_p->data[5])/1000 ;
      
        sig_p->x = frame_p->data[6];
        sig_p->y = frame_p->data[7];
        sig_p->z = frame_p->data[8];
        sig_p->a = frame_p->data[9];

        sig_p->cx = frame_p->data[10];
        sig_p->cy = frame_p->data[11];
        sig_p->cz = frame_p->data[12];

        sig_p->rotVector.X = frame_p->data[13];
        sig_p->rotVector.Y = frame_p->data[14];
        sig_p->rotVector.Z = frame_p->data[15];

        sig_p->turns = frame_p->data[16];

        sig_p->moveType = MOVE_ARC2;
        #if TEST_AXE_PIPELINE == 1
        printf("rec arc2, seq=%d\n",sig_p->seqNo);
        #endif

        sig_p->Send();

        frame_p->data[0] = CncAxeProcess_c::actSeqNo;
        frame_p->data[1] = lastReceivedSeqNo ;
        frame_p->size = 3;
      }
      return 8;

    case OC_RUN_AUTOBASE :
      {
        CNC_moveSig_c* sig_p = new CNC_moveSig_c;
        sig_p->x = frame_p->data[2];
        sig_p->y = frame_p->data[3];
        sig_p->z = frame_p->data[4];
        sig_p->a = frame_p->data[5];

        sig_p->moveType = MOVE_AUTOBASE;

        sig_p->Send();

        frame_p->size = 1;
      }
      return 8;
    case OC_SET_ZERO:
      {
        CNC_SetZeroSig_c* sig_p = new CNC_SetZeroSig_c;
        for(int i=0;i<NO_OF_AXES;i++)
        {
          sig_p->zeroOffset[i] = frame_p->data[i];
        }

        sig_p->Send();

        frame_p->size = 1;
      }
      return 4;
    case OC_RUN_DELAY:
      {
        CNC_moveSig_c* sig_p = new CNC_moveSig_c;
        sig_p->seqNo = frame_p->data[0];
        sig_p->delay = frame_p->data[2];

        sig_p->moveType = MOVE_DELAY;

        sig_p->Send();

        frame_p->size = 1;
      }
      return 8;
    case OC_RUN_STOP:
    case OC_RUN_PAUSE:
      return 4;

    case OC_STOP:
      {
        CncAxeProcess_c::forceBreak = true;
        CNC_StopSig_c* sig_p = new CNC_StopSig_c;
        sig_p->Send();

      }
      return 4;
    case OC_RUN_PROBE:
      {
        CNC_moveSig_c* sig_p = new CNC_moveSig_c;
        sig_p->seqNo = frame_p->data[0];
        sig_p->axe = frame_p->data[2];
        sig_p->length = frame_p->data[3];
        sig_p->mode = frame_p->data[4];
        sig_p->maxSpeed = ((float)frame_p->data[5])/1000 ;
        sig_p->maxAcceleration = ((float)frame_p->data[6])/1000 ;

        sig_p->moveType = MOVE_PROBE;

        sig_p->Send();

        frame_p->size = 1;
      }
      return 8;
    case OC_RESULT_ACK:
      {
        uint32_t ackFLags = frame_p->data[0];
        CncStatus_c::ClearStatusBit(ackFLags);
      }
      return 4;
    case OC_SET_BASE:
      {

        CNC_SetBaseSig_c* sig_p = new CNC_SetBaseSig_c;
        sig_p->axe = frame_p->data[0];
        sig_p->offset = frame_p->data[1];
        sig_p->Send();

      }
      return 4;

    case OC_SET_CONFIG:
      {
        CNC_SetConfig_c* sig_p = new CNC_SetConfig_c;

        int axeConfSize = 4;
        int offset2 = NO_OF_AXES * axeConfSize;

        for(int i=0;i<NO_OF_AXES;i++)
        {
          int offset = axeConfSize * i;
          sig_p->axeScale[i] = frame_p->data[offset];
          int map = frame_p->data[offset + 1];
          sig_p->axeEna[i] = ((map & 0x0001) > 0);
          sig_p->axeDir[i] = ((map & 0x0002) > 0);
          sig_p->limMode[i] = (LIMITER_MODE_et)((map >> 8) & 0x03);
          sig_p->limType[i] = (LIMITER_TYPE_et)((map >> 12) & 0x03);
          sig_p->maxSpeed[i] = ((float)frame_p->data[offset+2])/1000 ;
          sig_p->maxAcc[i] = ((float)frame_p->data[offset+3])/1000 ;
        }
        sig_p->estopMode = frame_p->data[offset2];
        sig_p->probeMode = frame_p->data[offset2+1];
        sig_p->minSpeed = ((float)frame_p->data[offset2+2])/1000 ;
        sig_p->autoBaseSpeed = ((float)frame_p->data[offset2+3])/1000 ;
        sig_p->spindleMaxSpeed = frame_p->data[offset2+4];

        sig_p->Send();


      }
      return 4;

    case OC_SET_SPINDLE_SPEED:
      {
        CNC_moveSig_c* sig_p = new CNC_moveSig_c;
        sig_p->seqNo = frame_p->data[0];
        sig_p->spindleSpeed = frame_p->data[2];

        sig_p->moveType = MOVE_SET_SPINDLE;

        sig_p->Send();

        frame_p->size = 1;
      }
      return 8;

    case OC_MANUAL_MOVE_START:
      {
        CNC_ManualMove_c * sig_p = new CNC_ManualMove_c;
        sig_p->axe = frame_p->data[2];
        sig_p->direction = frame_p->data[3];
        sig_p->maxSpeed = ((float)frame_p->data[4])/1000 ;
        sig_p->maxAcc = ((float)frame_p->data[5])/1000 ;
        sig_p->start = true;
        sig_p->Send();
        frame_p->size = 1;

      }
      return 8;

    case OC_MANUAL_MOVE_STOP:
      {
        CNC_ManualMove_c * sig_p = new CNC_ManualMove_c;
        sig_p->start = false;
        sig_p->Send();
        frame_p->size = 1;

      }
      return 8;
    case OC_SUFRACE_OFFSET:
      {
        CNC_SurfaceOffsetSig_c * sig_p = new CNC_SurfaceOffsetSig_c;
        sig_p->orderCode = (CNC_SurfaceOffsetSig_c::OrderCode_et)frame_p->data[0];
        sig_p->x = frame_p->data[1];
        sig_p->y = frame_p->data[2];
        sig_p->xStep = frame_p->data[3];
        sig_p->yStep = frame_p->data[4];
        sig_p->xStart = frame_p->data[5];
        sig_p->yStart = frame_p->data[6];
        sig_p->val = frame_p->data[7];
        sig_p->Send();
        frame_p->size = 1;

      }
      return 4;

    default:
      frame_p->size = 0;
      return 4;

  }


}



void CNC2_socket_c::HandlePacket(uint8_t* packet_p,uint16_t packetSize)
{
  //printf("Handle Rx DHCP \n");

  Packet_st* packet = (Packet_st*) packet_p;
  uint32_t ip = ntohl(packet->ipHeader.srcIP);
  uint16_t port = ntohs(packet->udpHeader.srcPort);

  CncFrame_st* message_p = (CncFrame_st*) &(packet->udpPayload);

  for(int i=0;i< message_p->size; i++)
  {
    message_p->data[i] = ntohl(message_p->data[i]);
  }

  uint16_t dataSize = ResponseMessage(message_p,1);

  for(int i=0;i< message_p->size; i++)
  {
    message_p->data[i] = htonl(message_p->data[i]);
  }

  uint16_t headersSize = PrepareHeaders(packet,dataSize,port,packet->macHeader.MAC_Src,ip);

  packetSize = headersSize + dataSize;

  SendPacket(packet_p,packetSize,nullptr,0);
  

 EthernetBuffers_c::DeleteBuffer(packet_p);

}





