#ifndef CNC2_AXE_PROCESS_H
#define CNC2_AXE_PROCESS_H

#include "SignalList.hpp"
#include "CommonDef.hpp"


#include "RngClass.hpp"
#include "TimeClass.hpp"

#include "CNC2_axe.hpp"
#include "Vector.hpp"

#include "CNC2_dataDef.hpp"
#include "CNC2_Segment.hpp"

#include "CNC2_SurfaceOffset.hpp"


#define STATUS_BIT_NOT_INIT       0x00001000
#define STATUS_BIT_NOT_BASED      0x00002000
#define STATUS_BIT_STOPPED        0x00004000
#define STATUS_BIT_X_LIM          0x00008000
#define STATUS_BIT_Y_LIM          0x00010000
#define STATUS_BIT_Z_LIM          0x00020000
#define STATUS_BIT_A_LIM          0x00040000
#define STATUS_BIT_B_LIM          0x00080000
#define STATUS_BIT_ESTOP          0x00100000
#define STATUS_BIT_PROBE_ERROR    0x00200000
/*0x00400000*/
/*0x00800000*/
#define STATUS_BIT_PROBE_END      0x01000000
#define STATUS_BIT_MMOVE_END      0x02000000
#define STATUS_BIT_PROBE_RESULT   0x04000000
#define STATUS_BIT_BASE_RESULT    0x08000000


#define STATUS_BIT_LIMITERS (STATUS_BIT_X_LIM | STATUS_BIT_Y_LIM | STATUS_BIT_Z_LIM | STATUS_BIT_A_LIM |STATUS_BIT_B_LIM )
#define STATUS_BIT_ERRORS (STATUS_BIT_NOT_INIT | STATUS_BIT_STOPPED| STATUS_BIT_ESTOP | STATUS_BIT_PROBE_ERROR)

enum MoveType_et
{
  MOVETYPE_REGULAR,
  MOVETYPE_BASE,
  MOVETYPE_PROBE,
  MOVETYPE_MANUAL
};

enum RunState_et
{
  RUN_IDLE,
  RUN_RUNNING, 
  RUN_SLOWING,
  RUN_STOPPING
};

struct RunData_st
{

  uint32_t calCnt[AXE_NOOF];
  uint32_t pulsesLeft[AXE_NOOF];

  float actSpeed;

  Segment_c* actSegment_p;

  MoveType_et regularMove;

  RunState_et state;
  float dV;
  float maxV;

};

class CncStatus_c
{
  static uint32_t status;

  public:

  static uint32_t GetStatus(void)
  {
    return status;
  }

  static void SetStatusBit(uint32_t bitMasc)
  {
    status |= bitMasc;
  }

  static void ClearStatusBit(uint32_t bitMasc)
  {
    status &= ~bitMasc;
  }

  static void WriteStatusBit(uint32_t bitMasc, bool state)
  {
    if(state)
    {
      SetStatusBit(bitMasc);
    }
    else
    {
      ClearStatusBit(bitMasc);
    }
  }

  static bool StatusOk(void)
  {
    uint32_t statusTMp = GetStatus();
    statusTMp &= (STATUS_BIT_ERRORS | STATUS_BIT_LIMITERS | STATUS_BIT_NOT_BASED);
    return statusTMp==0;
  }

  static bool StatusOkNoLimit(void)
  {
    uint32_t statusTMp = GetStatus();
    statusTMp &= (STATUS_BIT_ERRORS | STATUS_BIT_NOT_BASED);
    return statusTMp==0;
  }

    static bool StatusOkNoBase(void)
  {
    uint32_t statusTMp = GetStatus();
    statusTMp &= (STATUS_BIT_ERRORS | STATUS_BIT_LIMITERS);
    return statusTMp==0;
  }

};

class CncAxes_c
{

  

  public:

  Axe_c* axe_p[AXE_NOOF];


  RunData_st runData;

  int actSegIdx;

  void CalcSegment(Segment_c* segment_p,iVector3D* vector_p,int aMove,float v0, float vE, float maxSpeed, float maxAcc);

  void CalcSpeeds(Segment_c* segment_p,float length, float v0, float vE, float vMax, float maxA);

  

  CncAxes_c(void);

  void Init();

};



class CncAxeProcess_c : public process_c
{

  
 

  float autoBaseSpeed;
  int spindleMaxSpeed;

  SurfaceOffset_c surfaceOffset;

  static bool restartFinished;

  
  void HandleMove(CNC_moveSig_c* recSig_p);

  void RunBase(AXE_et axe, DIR_et dir);

  void RunProbe(CNC_moveSig_c* recSig_p);

  void SurfaceOffset(CNC_SurfaceOffsetSig_c* recSig_p);

  void StartMoving(void);
 
  void HandleManualMove( CNC_ManualMove_c* recSig_p);
  static void FlushQueue(void);

  public :

  static bool ignoreLimiters;

  static int pos[NO_OF_AXES];
  static int actSeqNo;


  static bool forceBreak;

  CncAxeProcess_c(uint16_t stackSize, uint8_t priority, uint8_t queueSize, HANDLERS_et procId);

  void main(void);

  static void CheckLimiters( bool fromISR);
  static void CheckEmergencyStop(void);

  static void DebugSpeed(float speed, int event);

  static void PrintSpeedDebug(void);

  static void CriticalChecksTick(void);

};





#endif