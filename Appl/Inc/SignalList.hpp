#ifndef SIGNALLIST_C
#define SIGNALLIST_C

class SignalList_c
{

  public: enum HANDLERS_et
  {
    HANDLE_CTRL,
    HANDLE_TCP,
    HANDLE_HTTP,
    HANDLE_TELNET,
    HANDLE_FTP,

    HANDLE_AXE,
    HANDLE_NO_OF

  };


  public: enum SIGNO_et
  {
    SIGNO_TEST = 0,
    SIGNO_SDIO_CardDetectEvent = 1,

    SIGNO_WATCHDOG_RESET,

    SIGNO_TCP_LINKEVENT,
    SIGNO_TCP_RXEVENT,
    SIGNO_TCP_DHCP_TIMEOUT,
    SIGNO_TCP_SEND_TIMER,
    SIGNO_TCP_TICK,
    SIGNO_SOCKET_SEND_REQUEST,
    SIGNO_SOCKET_REC_REQUEST,
    SIGNO_SOCKET_TCP_REQUEST,
    SIGNO_SOCKET_ADD,
    SIGNO_IP_CHANGED,

    SIGNO_CNC_MOVE,
    SIGNO_CNC_STOP,
    SIGNO_CNC_SETBASE,
    SIGNO_CNC_SET_CONFIG,
    SIGNO_CNC_RESET,
    SIGNO_CNC_SETZERO,

    SIGNO_CNC_MANUAL_MOVE,
    SINGO_CNC_RESET_ALL,

    SIGNO_CNC_SUFRACE_OFFSET,

  };
};

#include "commonSignal.hpp"

class testSig_c : public Sig_c
{
  public:
  testSig_c(void) : Sig_c(SIGNO_TEST,HANDLE_CTRL) {}

};

class watchdogSig_c : public Sig_c
{
  public:
  watchdogSig_c(void) : Sig_c(SIGNO_WATCHDOG_RESET,HANDLE_AXE) {}

};

class tcpRxEventSig_c: public Sig_c
{
  public:
  tcpRxEventSig_c(void) : Sig_c(SIGNO_TCP_RXEVENT,HANDLE_TCP) {dataBuffer = nullptr;}

  uint8_t* dataBuffer;
  uint32_t dataSize;

};

class tcpLinkEventSig_c: public Sig_c
{
  public:
  tcpLinkEventSig_c(void) : Sig_c(SIGNO_TCP_LINKEVENT,HANDLE_TCP) {}

  uint8_t linkState;

};

class tcpDhcpTimerSig_c: public Sig_c
{
  public:
  tcpDhcpTimerSig_c(void) : Sig_c(SIGNO_TCP_DHCP_TIMEOUT,HANDLE_TCP) {}

  uint8_t timerIndicator;

};


class tcpSendSig_c : public Sig_c
{
  public:
  tcpSendSig_c(void) : Sig_c(SIGNO_TCP_SEND_TIMER,HANDLE_TCP) {}

};

class tcpTickSig_c : public Sig_c
{
  public:
  tcpTickSig_c(void) : Sig_c(SIGNO_TCP_TICK,HANDLE_TCP) {}

};

#include "TcpDataDef.hpp"
class socketSendReqSig_c : public Sig_c
{
  public:
  socketSendReqSig_c(void) : Sig_c(SIGNO_SOCKET_SEND_REQUEST,HANDLE_TCP) {}
  SocketTcp_c* socket;
  TaskHandle_t task;
  uint32_t bufferSize;
  uint32_t bytesSent;
  uint8_t* buffer_p;

};

class socketReceiveReqSig_c : public Sig_c
{
  public:
  socketReceiveReqSig_c(void) : Sig_c(SIGNO_SOCKET_REC_REQUEST,HANDLE_TCP) {}
  SocketTcp_c* socket;
  TaskHandle_t task;
  uint32_t bufferSize;
  uint32_t bytesReceived;
  uint8_t* buffer_p;

};


class socketTcpRequestSig_c: public Sig_c
{
  public:
  SocketRequest_et code;
  socketTcpRequestSig_c(void) : Sig_c(SIGNO_SOCKET_TCP_REQUEST,HANDLE_TCP) {}
  SocketTcp_c* socket;
  TaskHandle_t task;

  SocketAdress_st* soccAdr;
  uint32_t bufferSize;
  uint8_t* buffer_p;
  uint8_t clientMaxCnt;
};

class ipChanged_c : public Sig_c
{
  public:
  ipChanged_c(void) : Sig_c(SIGNO_IP_CHANGED,HANDLE_TCP) {}

};


class socketAddSig_c : public Sig_c
{
  public:
  socketAddSig_c(void) : Sig_c(SIGNO_SOCKET_ADD,HANDLE_TCP) {}
  class Socket_c* socket;
};

/****************** CNC SIGNALS ********************************/
#include "CNC2_def.hpp"
#include "Vector.hpp"

class CNC_moveSig_c : public Sig_c
{
  public:
  CNC_moveSig_c(void) : Sig_c(SIGNO_CNC_MOVE,HANDLE_AXE) 
  {
    ignoreLimiters = false;
  }

  float speedStart;
  float speedEnd;
  float maxSpeed;
  float maxAcceleration;


  MOVE_TYPE_et moveType;

  int x,y,z,a;
  int cx,cy,cz;
  
  Vector3D rotVector;



  bool ignoreLimiters;


  PLANE_SELECT_et plane;
  int turns;
  bool clockwise;


  int seqNo;

  union{
  int delay;
  int spindleSpeed;
  int length;
  };
  uint8_t axe;
  uint8_t mode;
  
};

class CNC_StopSig_c : public Sig_c
{
  public:
  CNC_StopSig_c(void) : Sig_c(SIGNO_CNC_STOP,HANDLE_AXE) {}
  
};

class CNC_ResetSig_c : public Sig_c
{
  public:
  CNC_ResetSig_c(void) : Sig_c(SIGNO_CNC_RESET,HANDLE_AXE) {}
  
};

class CNC_SetBaseSig_c : public Sig_c
{
  public:
  CNC_SetBaseSig_c(void) : Sig_c(SIGNO_CNC_SETBASE,HANDLE_AXE) {}

  int axe;
  int offset;
  
};

class CNC_SetConfig_c : public Sig_c
{
  public:
  CNC_SetConfig_c(void) : Sig_c(SIGNO_CNC_SET_CONFIG,HANDLE_AXE) {}

  bool axeEna[NO_OF_AXES];
  bool axeDir[NO_OF_AXES];
  int axeScale[NO_OF_AXES];
  enum LIMITER_MODE_et limMode[NO_OF_AXES];
  enum LIMITER_TYPE_et limType[NO_OF_AXES];
  float maxSpeed[NO_OF_AXES];
  float maxAcc[NO_OF_AXES];
  int estopMode;
  int probeMode;
  float minSpeed;
  float autoBaseSpeed;
  int spindleMaxSpeed;
  
};


class CNC_SetZeroSig_c : public Sig_c
{
  public:
  CNC_SetZeroSig_c(void) : Sig_c(SIGNO_CNC_SETZERO,HANDLE_AXE) {}
  int zeroOffset[NO_OF_AXES];
  
};

class CNC_ManualMove_c : public Sig_c
{
  public:
  CNC_ManualMove_c(void) : Sig_c(SIGNO_CNC_MANUAL_MOVE,HANDLE_AXE) {}

  uint8_t axe;
  int direction;
  float maxAcc;
  float maxSpeed;
  bool start;
  
};

class CNC_ResetAllSig_c : public Sig_c
{
  public:
  CNC_ResetAllSig_c(void) : Sig_c(SINGO_CNC_RESET_ALL,HANDLE_AXE) {}
  
};



class CNC_SurfaceOffsetSig_c : public Sig_c
{
  public:
  CNC_SurfaceOffsetSig_c(void) : Sig_c(SIGNO_CNC_SUFRACE_OFFSET,HANDLE_AXE) {}

  enum OrderCode_et
  {
    INIT,
    SET_POINT,
    ACTIVATE,
    DEACTIVATE,
    CLEAR
  } orderCode;

  int x;
  int y;
  int xStep;
  int yStep;
  int xStart;
  int yStart;
  int val;  
};









#endif