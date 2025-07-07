 #include <stdio.h>
#include <stdlib.h>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "math.h"


#include "CNC2_axeProcess.hpp"
#include "CNC2_MoveProcessor.hpp"

#include "Matrix.hpp"

extern TIM_HandleTypeDef AXE_X_TIMER;
extern TIM_HandleTypeDef AXE_Y_TIMER;
extern TIM_HandleTypeDef AXE_Z_TIMER;
extern TIM_HandleTypeDef AXE_A_TIMER;
extern TIM_HandleTypeDef AXE_B_TIMER;

CncAxes_c* axes_p = nullptr;

CNC_ResetAllSig_c resetSig;

#if USE_AUTOBASE == 1
uint32_t CncStatus_c::status = STATUS_BIT_NOT_INIT | STATUS_BIT_NOT_BASED;
#else
uint32_t CncStatus_c::status = STATUS_BIT_NOT_INIT ;
#endif


int CncAxeProcess_c::pos[NO_OF_AXES] = {0,0,0,0};
int CncAxeProcess_c::actSeqNo = -1;

bool CncAxeProcess_c::forceBreak = false;
bool CncAxeProcess_c::ignoreLimiters = false;
bool CncAxeProcess_c::restartFinished = false;

bool interruptCheck[NO_OF_AXES] = {false,false,false,false};

int segmentCnt = 0;

watchdogSig_c wtdSig;

#if CONF_USE_WATCHDOG == 1
extern IWDG_HandleTypeDef hiwdg1;
#endif

#if TEST_SEGLIST == 1

#define DEBUG_DATA_SIZE 4
struct DebugData_st
{
  int wantedPos[NO_OF_AXES];
  int realPos[NO_OF_AXES];
  int seqNo;
  int segmentNr;

  uint32_t pulses[AXE_NOOF];
  DIR_et dir[AXE_NOOF];

  
};
DebugData_st debugData[DEBUG_DATA_SIZE];
int debugDataIdx = 0;


bool SaveDebugData(Segment_c* segment_p)
{
  bool allOk = true;
  int diffPos[NO_OF_AXES];
  int maxDiff = 0;
  DebugData_st* p = &debugData[debugDataIdx];
  for(int i=0;i<NO_OF_AXES;i++)
  {
    p->realPos[i] = CncAxeProcess_c::pos[i];
    p->wantedPos[i] = axes_p->runData.actSegment_p->endPos[i];
    diffPos[i] = CncAxeProcess_c::pos[i] - axes_p->runData.actSegment_p->endPos[i];
    int absDiff = abs(diffPos[i]);
    if(absDiff > maxDiff) { maxDiff = absDiff; }

    p->pulses[i] = segment_p->pulses[i];
    p->dir[i] =  segment_p->dir[i];
  }
  if(maxDiff > 50) {  allOk = false; }

  
  p->seqNo = axes_p->runData.actSegment_p->seqNo;
  p->segmentNr = axes_p->runData.actSegment_p->segmentNr;

  debugDataIdx++;
  debugDataIdx &= (DEBUG_DATA_SIZE-1);
  return allOk;
}

void PrintDebugData(void)
{
  int nr = 0;

  int idx = debugDataIdx ;

  bool cont = true;
  while (cont)
  {
    DebugData_st* p = &debugData[idx];
    printf("IDX=%d, seqNo=%d, segNr=%d\n",nr,p->seqNo,p->segmentNr);

    printf("Real   pos = (%d,%d,%d,%d)\n",p->realPos[0],p->realPos[1],p->realPos[2],p->realPos[3]);
    printf("Wanted pos = (%d,%d,%d,%d)\n",p->wantedPos[0],p->wantedPos[1],p->wantedPos[2],p->wantedPos[3]);
    printf("Difference = (%d,%d,%d,%d)\n",p->realPos[0]-p->wantedPos[0],p->realPos[1]-p->wantedPos[1],p->realPos[2]-p->wantedPos[2],p->realPos[3]-p->wantedPos[3]);

    printf("Pulses = %d,%d,%d,%d\n",p->pulses[0],p->pulses[1],p->pulses[2],p->pulses[3]);
    printf("Dir = %d,%d,%d,%d\n",p->dir[0],p->dir[1],p->dir[2],p->dir[3]);

    idx++;
    idx &= (DEBUG_DATA_SIZE-1);
    nr++;

    if( idx == debugDataIdx) { cont = false; }
  } ;




}
#endif


void vFunctionAxeTimerCallback( TimerHandle_t xTimer )
{
#if USE_WATCHDOG == 1
  //wtdSig.Send();
  __HAL_IWDG_RELOAD_COUNTER(&hiwdg1);
  #endif

}

void CncAxeProcess_c::CriticalChecksTick(void)
{
  if(restartFinished)
  {
    CncAxeProcess_c::CheckEmergencyStop();
    CncAxeProcess_c::CheckLimiters(true);
  }
}


CncAxeProcess_c::CncAxeProcess_c(uint16_t stackSize, uint8_t priority, uint8_t queueSize, HANDLERS_et procId) : process_c(stackSize,priority,queueSize,procId,"CNC_AXE")
{

  TimerHandle_t timer = xTimerCreate("",pdMS_TO_TICKS(100),pdTRUE,( void * ) 0,vFunctionAxeTimerCallback);
  xTimerStart(timer,0);


 

 
  autoBaseSpeed = 1;
  spindleMaxSpeed = 18000;


 //MatrixTest();

}
/*
float debugSpeedHistory[32];
int debugSpeedEvent[32];
int debugSpeedIdx = 0;

*/
void CncAxeProcess_c::DebugSpeed(float speed, int event)
{
/*
  debugSpeedHistory[debugSpeedIdx] = speed;
   debugSpeedHistory[debugSpeedIdx] = event;
  debugSpeedIdx++;
  if(debugSpeedIdx>= 32) 
  {  
    debugSpeedIdx = 0;
  }*/

}

void CncAxeProcess_c::PrintSpeedDebug(void)
{
 /* int tmpIdx = debugSpeedIdx;

  do
  {
    tmpIdx++;
    if(tmpIdx>= 32) 
    {  
      tmpIdx = 0;
    }
    printf("Speed = %f, event = %d\n", debugSpeedHistory[tmpIdx], debugSpeedHistory[tmpIdx]);


  }
  while(tmpIdx != debugSpeedIdx);

*/

}

void CncAxeProcess_c::main(void)
{

  #if DEBUG_PROCESS > 0
  printf("CNC Axe proc started \n");
  #endif



  axes_p = new CncAxes_c;
  axes_p->Init();

  Segment_c::InitList();
  FlushQueue();

  Axe_c::SetSpindleSpeed(0);

  restartFinished = true;




  while(1)
  {
    releaseSig = true;
    RecSig();
    uint8_t sigNo = recSig_p->GetSigNo();

    if(forceBreak) 
    {
      CncStatus_c::SetStatusBit(STATUS_BIT_STOPPED);
    }

    switch(sigNo)
    {
   
      case SIGNO_WATCHDOG_RESET:
        {
         #if USE_WATCHDOG == 1
          //__HAL_IWDG_RELOAD_COUNTER(&hiwdg1);
         #endif
          releaseSig = false;
        }
        break;
        
      case SIGNO_CNC_MOVE:
        {
          CNC_moveSig_c* sig_p = (CNC_moveSig_c*)recSig_p;
          uint32_t statusBitMap = CncStatus_c::GetStatus();
          ignoreLimiters = sig_p->ignoreLimiters;
          if(CncStatus_c::StatusOk() == true)
          {
            HandleMove(sig_p);
          }
          else if((CncStatus_c::StatusOkNoLimit() == true) && (ignoreLimiters == true))
          {
            HandleMove(sig_p);
          }
          else if((CncStatus_c::StatusOkNoBase() == true) && (sig_p->moveType == MOVE_AUTOBASE))
          {
            HandleMove(sig_p);
          }
        }
      break;

      case SIGNO_CNC_STOP:
        { 

          FlushQueue();

          Axe_c::MasterStop();
          Axe_c::SpeedStop();
          Axe_c::SetSpindleSpeed(0);
          forceBreak = false;

         

        }
      break;

      case SIGNO_CNC_SETBASE:
        if(CncStatus_c::StatusOk() == true)
        {
          CNC_SetBaseSig_c* sig_p = (CNC_SetBaseSig_c*)recSig_p;          
          CncAxeProcess_c::pos[sig_p->axe] = sig_p->offset;
          MoveProcessor_c::SetPosition(sig_p->axe,sig_p->offset);
        }

      break;

      case SIGNO_CNC_SETZERO:
        if(CncStatus_c::StatusOk() == true)
        {
          
          CNC_SetZeroSig_c* sig_p = (CNC_SetZeroSig_c*)recSig_p;
          for(int axe=0;axe<NO_OF_AXES;axe++)  
          {     
            CncAxeProcess_c::pos[axe] = sig_p->zeroOffset[axe];
            MoveProcessor_c::SetPosition(axe, sig_p->zeroOffset[axe]);
          }
        }

      break;


      case SIGNO_CNC_RESET:
      {
        FlushQueue();

        for(int i=0;i<NO_OF_AXES;i++)
        {
          MoveProcessor_c::SetPosition(i, pos[i]);
        }

        bool state = Axe_c::GetEStopState();
        if(state == true)
        {
          ignoreLimiters = true;
          CheckLimiters(false);
          forceBreak = false;
          CncStatus_c::ClearStatusBit(STATUS_BIT_STOPPED | STATUS_BIT_ESTOP | STATUS_BIT_PROBE_ERROR);
        }

        

        break; 
        }       

      case SIGNO_CNC_SET_CONFIG:
        {
        CNC_SetConfig_c* sig_p = (CNC_SetConfig_c*)recSig_p;
        for(int i=0;i<NO_OF_AXES;i++)
        {
          axes_p->axe_p[i]->SetConfig(sig_p->axeEna[i],sig_p->axeDir[i],sig_p->axeScale[i],sig_p->limMode[i],sig_p->limType[i], sig_p->maxSpeed[i], sig_p->maxAcc[i]);
        }
        Axe_c::SetEstopConfig(sig_p->estopMode);
        Axe_c::SetProbeConfig(sig_p->probeMode);
        Axe_c::SetMiscConfig(sig_p->minSpeed);
        autoBaseSpeed = sig_p->autoBaseSpeed;
        spindleMaxSpeed = sig_p->spindleMaxSpeed;

        
        CheckLimiters(false);

        CncStatus_c::ClearStatusBit(STATUS_BIT_NOT_INIT);

        }
      break;

      case SIGNO_CNC_MANUAL_MOVE:
        ignoreLimiters = false;
        HandleManualMove( (CNC_ManualMove_c*) recSig_p);
        break;
      
      case SINGO_CNC_RESET_ALL:
        FlushQueue();
        releaseSig = false;

        break;

      case SIGNO_CNC_SUFRACE_OFFSET:
        SurfaceOffset((CNC_SurfaceOffsetSig_c*) recSig_p);
      break;

      default:
      break;

    }
    if(releaseSig)
    {
      delete  recSig_p;
    } 
  }
}

void CncAxeProcess_c::HandleMove(CNC_moveSig_c* recSig_p)
{
  MoveData_st move;
  
  MoveProcData_st runData;

  int scales[AXE_NOOF];

  if(axes_p->actSegIdx != -1)
  {
    if(axes_p->actSegIdx +1 != recSig_p->seqNo)
    {
      printf("seq error, prev=%d, act=%d\n",axes_p->actSegIdx, recSig_p->seqNo);
    }
    axes_p->actSegIdx = recSig_p->seqNo;
  }

  for(int i=0;i<AXE_NOOF;i++)
  {
    scales[i] = axes_p->axe_p[i]->GetScale();
  }

  switch(recSig_p->moveType)
  {
    case MOVE_LINE:

      MoveProcessor_c::InitLine(&runData, recSig_p, surfaceOffset.GetMaxSegLength() );
      break;

    case MOVE_ARC:
      
      move.moveType = recSig_p->moveType;

  /* position data */
      move.centrePoint = iPoint3D(recSig_p->cx,recSig_p->cy,recSig_p->cz);
      move.endPoint = iPoint3D(recSig_p->x,recSig_p->y,recSig_p->z);
      move.endApos = recSig_p->a;
      move.planeMode = recSig_p->plane;
      move.clockwise = recSig_p->clockwise;
      move.turns = recSig_p->turns;
  
  /* speed data */
      move.maxA = recSig_p->maxAcceleration;
      move.startSpeed = recSig_p->speedStart;
      move.endSpeed = recSig_p->speedEnd;
      move.maxSpeed  =recSig_p->maxSpeed;

      MoveProcessor_c::InitArc(&runData, &move);

      break;

   case MOVE_ARC2:
      MoveProcessor_c::InitArcNew(&runData, recSig_p);
      break;

   case MOVE_DELAY:

      move.moveType = recSig_p->moveType;
      move.delay = recSig_p->delay;
      MoveProcessor_c::InitDwel(&runData, &move);
      break;

   case MOVE_SET_SPINDLE:

     runData.moveType = recSig_p->moveType;
     runData.spindleSpeed = (recSig_p->spindleSpeed * 1000)/spindleMaxSpeed;
     break;

   case MOVE_AUTOBASE:

      if(recSig_p->z > 0) {  RunBase(AXE_Z,DIR_UP);  }
      else if(recSig_p->z < 0) {  RunBase(AXE_Z,DIR_DOWN);  }

      if(recSig_p->x > 0) {  RunBase(AXE_X,DIR_UP);  }
      else if(recSig_p->x < 0) {  RunBase(AXE_X,DIR_DOWN);  }

      if(recSig_p->y > 0) {  RunBase(AXE_Y,DIR_UP);  }
      else if(recSig_p->y < 0) {  RunBase(AXE_Y,DIR_DOWN);  }

      CncStatus_c::ClearStatusBit(STATUS_BIT_NOT_BASED);      

      CncStatus_c::SetStatusBit(STATUS_BIT_BASE_RESULT);

      return;
   case MOVE_PROBE:
      RunProbe(recSig_p);
      return;

   default:
      break;



  }
  int segCnt = 0;


  SEGMENT_REULT_et segResult = MORE_SEGMENTS;
  while((segResult == MORE_SEGMENTS) || (segResult == IGNORE_SEGMENT))
  {
    #if USE_WATCHDOG == 1
    //__HAL_IWDG_RELOAD_COUNTER(&hiwdg1);
    #endif
    if(forceBreak)
    {
      break;
    }

    Segment_c* segment_p = Segment_c::GetEmpty();
    while(segment_p == nullptr)
    {
      /*  delay */
      vTaskDelay(10);
      segment_p = Segment_c::GetEmpty();
    }

    switch(runData.moveType)
    {
      case MOVE_LINE:
      {
        SimpleMove_st simpleMove;
        segResult = MoveProcessor_c::GetLineSimpleMove(&simpleMove,&runData,scales,&surfaceOffset);
        if(segResult != IGNORE_LAST_SEGMENT) 
        {
          segment_p->orderCode = MOVE_LINE;          
          iVector3D vector = iVector3D(simpleMove.x,simpleMove.y,simpleMove.z);
          axes_p->CalcSegment(segment_p,&vector, simpleMove.a, simpleMove.startSpeed, simpleMove.endSpeed, runData.vMax, runData.aMax  );

          for(int i=0;i<NO_OF_AXES; i++)
          {
            segment_p->endPos[i] = MoveProcessor_c::GetPosition(i);
          }

        }
        
      }
      break;
      case MOVE_ARC:
      {
        SimpleMove_st simpleMove;
        segResult = MoveProcessor_c::GetArcSimpleMove(&simpleMove,&runData,scales);
        segment_p->orderCode = MOVE_ARC;
        iVector3D vector = iVector3D(simpleMove.x,simpleMove.y,simpleMove.z);
        axes_p->CalcSegment(segment_p,&vector, simpleMove.a,simpleMove.startSpeed, simpleMove.endSpeed, runData.vMax, runData.aMax  );
        for(int i=0;i<NO_OF_AXES; i++)
        {
          segment_p->endPos[i] = MoveProcessor_c::GetPosition(i);
        }
     
      }
      break;

      case MOVE_ARC2:
      {
        SimpleMove_st simpleMove;
        segResult = MoveProcessor_c::GetArcNewSimpleMove(&simpleMove,&runData,scales,&surfaceOffset);
        if((segResult != IGNORE_LAST_SEGMENT) && (segResult != IGNORE_SEGMENT)) 
        {
          segment_p->orderCode = MOVE_ARC2;          
          iVector3D vector = iVector3D(simpleMove.x,simpleMove.y,simpleMove.z);
          axes_p->CalcSegment(segment_p,&vector, simpleMove.a,simpleMove.startSpeed, simpleMove.endSpeed, runData.vMax, runData.aMax  );
 
          for(int i=0;i<NO_OF_AXES; i++)
          {
            segment_p->endPos[i] = MoveProcessor_c::GetPosition(i);
          }

        }
        else
        {
          printf("segment ignored, seqNo=%d\n",recSig_p->seqNo);
        }

     
      }
      break;

      case MOVE_DELAY:
      {

        for(int i=0;i<AXE_NOOF;i++)
        {
          segment_p->period[i] = (PULSE_MULTIPLIER/1000);
          segment_p->calibTicks[i] = 0;
          segment_p->prescaler[i] = 0;
          segment_p->dir[i] = DIR_NONE;
          segment_p->pulses[i] = runData.delay;
        }
        segment_p->periodCalib = 0;
        segment_p->totalLength = runData.delay*(PULSE_MULTIPLIER/1000);
        segment_p->orderCode = MOVE_DELAY;
        segment_p->v0 = 1;
        segment_p->vM = 1;
        segment_p->vE = 1;
        

        for(int i=0;i<NO_OF_AXES; i++)
        {
          segment_p->endPos[i] = MoveProcessor_c::GetPosition(i);
        }

        segResult = LAST_SEGMENT;
     
      }
      break;
      case MOVE_SET_SPINDLE:
      {
        for(int i=0;i<AXE_NOOF;i++)
        {
          segment_p->period[i] = 100;
          segment_p->calibTicks[i] = 0;
          segment_p->prescaler[i] = 0;
          segment_p->dir[i] = DIR_NONE;
          segment_p->pulses[i] = 2;
        }
        segment_p->periodCalib = 0;
        segment_p->totalLength = 2*(PULSE_MULTIPLIER/1000);
        
        segment_p->value = runData.spindleSpeed;
        segment_p->orderCode = MOVE_SET_SPINDLE;
        segment_p->v0 = 1;
        segment_p->vE = 1;
        segResult = LAST_SEGMENT;

        for(int i=0;i<NO_OF_AXES; i++)
        {
          segment_p->endPos[i] = MoveProcessor_c::GetPosition(i);
        }
      }
      break;


     default:
        break;

    }
    segment_p->seqNo = recSig_p->seqNo;

    if((segResult != IGNORE_LAST_SEGMENT) && (segResult != IGNORE_SEGMENT)) 
    {
      segment_p->segmentNr = segmentCnt;
      segmentCnt++;
      segment_p->AddToList();
      if(axes_p->runData.actSegment_p == nullptr)
      {
        StartMoving();
      }
    }


    #if CONF_USE_WATCHDOG == 1
    __HAL_IWDG_RELOAD_COUNTER(&hiwdg1);
    #endif
/*
    if(segCnt <= 0)
    {
      if((segResult == IGNORE_SEGMENT) || (segResult == MORE_SEGMENTS) )
      {

        vTaskDelay(1);
        segCnt = 100;
      }
    }
    else
    {
      segCnt--;
    }*/

  }
}

void CncAxeProcess_c::SurfaceOffset(CNC_SurfaceOffsetSig_c* recSig_p)
{
  switch(recSig_p->orderCode)
  {
    case CNC_SurfaceOffsetSig_c::INIT:
    {
      surfaceOffset.Clear();
      surfaceOffset.Init(recSig_p->x,recSig_p->y,recSig_p->xStep,recSig_p->yStep,recSig_p->xStart,recSig_p->yStart);
    }
    break;
    case CNC_SurfaceOffsetSig_c::CLEAR:
    {
      surfaceOffset.Clear();
    }
    break;
    case CNC_SurfaceOffsetSig_c::ACTIVATE:
    {
      surfaceOffset.Activate();
    }
    break;
    case CNC_SurfaceOffsetSig_c::DEACTIVATE:
    {
      surfaceOffset.Deactivate();
    }
    break;
    case CNC_SurfaceOffsetSig_c::SET_POINT:
    {
      surfaceOffset.SetProbe(recSig_p->x,recSig_p->y,recSig_p->val);
    }
    break;
    default:
    break;
  }
}

void CncAxeProcess_c::RunProbe(CNC_moveSig_c* recSig_p)
{

  while(axes_p->runData.actSegment_p != nullptr)
  {
    vTaskDelay(10);
  }


  bool cont = true;

  uint8_t mode = recSig_p->mode;
  AXE_et axe = (AXE_et)recSig_p->axe;
  DIR_et dir;
  int length = recSig_p->length;
  uint8_t probeState;
  uint8_t wantedProbeState;

  int errorPosition;

  bool useErrorPosition = false;
  bool resultOk = true;

  if(Axe_c::ProbeActive() == true )
  {
    probeState = Axe_c::GetProbeState();


    if(length < 0) { dir = DIR_DOWN; }
    else if (length > 0) { dir = DIR_UP; }
    else {cont = false;}

    if(mode == 2 || mode == 3)
    {
      wantedProbeState = 1;
    }
    else
    {
      wantedProbeState = 0;
    }

    if(mode == 2 || mode == 4)
    {
      useErrorPosition = true;
    }
  }

  if(cont)
  {
    if(wantedProbeState == probeState)
    {
      cont = false;
      resultOk = false;
    }
  }

  if(cont)
  {
    Axe_c* axe_p = axes_p->axe_p[axe];

    errorPosition = pos[axe] + length;


    uint32_t period = 100*  axe_p->GetScale();

    for(int i=0;i<AXE_NOOF;i++)
    {
      axes_p->axe_p[i]->SetDir(DIR_NONE);
      axes_p->axe_p[i]->SetState(DIR_NONE,0);
      axes_p->axe_p[i]->SetPeriod(0,0,true);
    }

    axes_p->runData.regularMove = MOVETYPE_PROBE;
    axes_p->runData.state = RUN_RUNNING;


    axes_p->runData.maxV = recSig_p->maxSpeed;
    float dt = 0.001 *SPEED_PERIOD ;
    axes_p->runData.dV = recSig_p->maxAcceleration * dt;

    axe_p->SetDir(dir);
    axe_p->SetState(dir,period/2);
    axe_p->SetPeriod(period,0,true);

    axes_p->runData.actSpeed = axes_p->runData.dV;

    printf("start probing, dV=%f, maxV=%f\n",axes_p->runData.dV,axes_p->runData.maxV);
    Axe_c::MasterSetSpeed(axes_p->runData.actSpeed);
    Axe_c::SpeedRun();
    Axe_c::MasterRun();

    do
    {
      probeState = Axe_c::GetProbeState();

      if(forceBreak == true)
      {
        Axe_c::MasterStop();
        Axe_c::SpeedStop();
        return;
      }
      vTaskDelay(2);


      if((dir == DIR_DOWN) && ( pos[axe] <= errorPosition))
      {
        resultOk = false;
        break;
      }
      else if((dir == DIR_UP) && ( pos[axe] >= errorPosition))
      {
        resultOk = false;
        break;
      }
 
    }
    while(probeState != wantedProbeState );   

    axes_p->runData.state = RUN_SLOWING;

    while( axes_p->runData.state != RUN_IDLE)
    {
      vTaskDelay(2);
    }  
  }

  MoveProcessor_c::SetPosition(axe, CncAxeProcess_c::pos[axe]);

  if(resultOk)
  {
     CncStatus_c::SetStatusBit(STATUS_BIT_PROBE_END | STATUS_BIT_PROBE_RESULT);

  }
  else
  {
    if(useErrorPosition)
    {
       CncStatus_c::SetStatusBit(STATUS_BIT_PROBE_END | STATUS_BIT_PROBE_ERROR);
    }
    else
    {
       CncStatus_c::SetStatusBit(STATUS_BIT_PROBE_END);
    }
  }
}

void CncAxeProcess_c::CheckLimiters(bool fromISR)
{
  uint8_t stateX = axes_p->axe_p[AXE_X]->GetLimState();
  uint8_t stateY = axes_p->axe_p[AXE_Y]->GetLimState();
  uint8_t stateZ = axes_p->axe_p[AXE_Z]->GetLimState();

  CncStatus_c::WriteStatusBit(STATUS_BIT_X_LIM, stateX > 0 );
  CncStatus_c::WriteStatusBit(STATUS_BIT_Y_LIM, stateY > 0 );
  CncStatus_c::WriteStatusBit(STATUS_BIT_Z_LIM, stateZ > 0 );

  if(ignoreLimiters == false)
  {
    if((stateX | stateY | stateZ)>0)
    {
      uint32_t status = CncStatus_c::GetStatus();
      if((status & STATUS_BIT_STOPPED) == 0)
      {
        forceBreak = true;
        Axe_c::MasterStop();
        Axe_c::SpeedStop();
        Axe_c::SetSpindleSpeed(0);
        if(fromISR)
        {
          resetSig.SendISR();
        }
        else
        {
          resetSig.Send();
        }
      }
      CncStatus_c::SetStatusBit(STATUS_BIT_STOPPED);
    }
  }
}

void CncAxeProcess_c::CheckEmergencyStop(void)
{
  bool state = Axe_c::GetEStopState();
  

  if(state == false)
  {
    uint32_t status = CncStatus_c::GetStatus();
    if((status & STATUS_BIT_STOPPED) == 0)
    {
      forceBreak = true;
      Axe_c::MasterStop();
      Axe_c::SpeedStop();
      Axe_c::SetSpindleSpeed(0);
      resetSig.SendISR();
    }
    //FlushQueue();
    CncStatus_c::SetStatusBit(STATUS_BIT_STOPPED | STATUS_BIT_ESTOP);
  }

}


void CncAxeProcess_c::FlushQueue(void)
{
  Segment_c::ResetList();
  axes_p->runData.actSegment_p = nullptr;
  axes_p->actSegIdx = -1;


}

void CncAxeProcess_c::RunBase(AXE_et axe, DIR_et dir)
{  

  Axe_c* axe_p = axes_p->axe_p[axe]; 
  //printf("Start base, axe %d\n",axe);

  if(axe_p->LimActive() == false ) { return ; }

  uint8_t limState = axe_p->GetLimState();

  if(limState != 0) { return; }


  uint32_t period = 100 * axe_p->GetScale();

  for(int i=0;i<AXE_NOOF;i++)
  {
    axes_p->axe_p[i]->SetDir(DIR_NONE);
    axes_p->axe_p[i]->SetState(DIR_NONE,0);
    axes_p->axe_p[i]->SetPeriod(0,0,true);
  }

  axes_p->runData.regularMove = MOVETYPE_BASE;
  axes_p->runData.state = RUN_RUNNING;

  axes_p->runData.maxV = autoBaseSpeed;
  if(axes_p->runData.maxV > axe_p->GetMaxSpeed()) { axes_p->runData.maxV = axe_p->GetMaxSpeed(); }
  float dt = 0.001 *SPEED_PERIOD ;
  axes_p->runData.dV = axe_p->GetMaxAcc() * dt;
  axes_p->runData.actSpeed = axes_p->runData.dV; 

  ignoreLimiters = true; 

  axe_p->SetDir(dir);
  axe_p->SetState(dir,period/2);
  axe_p->SetPeriod(period,0,true);

  Axe_c::MasterSetSpeed(axes_p->runData.actSpeed);
  Axe_c::SpeedRun();
  Axe_c::MasterRun();

  int cnt = 0;

  do
  {
    limState = axe_p->GetLimState();
    cnt++;
    if(forceBreak == true)
    {
      Axe_c::MasterStop();
      return;
    }
    vTaskDelay(2);
  }
  while(limState ==  0 );   

   // printf("lim reached, axe %d cnt=%d\n",axe,cnt);

  axes_p->runData.state = RUN_SLOWING;

  while( axes_p->runData.state != RUN_IDLE)
  {
    vTaskDelay(2);
  }

  //printf("axe stopped\n",axe,cnt);

  //printf("Start base 2, axe %d\n",axe);

  vTaskDelay(1000);

  switch(dir)
  {
    case DIR_UP : axe_p->SetDir(DIR_DOWN); axe_p->SetState(DIR_DOWN,period/2); break;
    case DIR_DOWN : axe_p->SetDir(DIR_UP); axe_p->SetState(DIR_UP,period/2); break;
    default: return;
  }
  
  //printf("Cont base 2, axe %d\n",axe);

  axes_p->runData.maxV = axes_p->runData.dV;
  axes_p->runData.actSpeed = axes_p->runData.dV; 
  
  axe_p->SetPeriod(period,0,true);

  Axe_c::MasterSetSpeed(axes_p->runData.actSpeed);
  Axe_c::MasterRun();

  cnt = 0;
  do
  {
    limState = axe_p->GetLimState();
    cnt++;
    if(forceBreak == true)
    {
      Axe_c::MasterStop();
      return;
    }
    vTaskDelay(2);
  }
  while(limState != 0 ); 

  //printf("lim leaved, axe %d cnt=%d\n",axe,cnt);

  axes_p->runData.state = RUN_STOPPING;

  while( axes_p->runData.state != RUN_IDLE)
  {
    vTaskDelay(2);
  }
  ignoreLimiters = false;

  //printf("axe stopped2\n",axe,cnt);


}

void CncAxeProcess_c::StartMoving(void)
{
HAL_GPIO_WritePin(TEST6_PORT,TEST6_PIN,GPIO_PIN_SET);
  Segment_c* segPtr = Segment_c::GetFromList();
  if(segPtr != nullptr)
  { 
    axes_p->runData.actSegment_p = segPtr;
    axes_p->runData.regularMove = MOVETYPE_REGULAR;

    if(segPtr->orderCode == MOVE_SET_SPINDLE)
    {
      Axe_c::SetSpindleSpeed(segPtr->value);
    }

    for(int i=0;i< AXE_NOOF;i++)
    {
      axes_p->axe_p[i]->Reset();
      axes_p->axe_p[i]->SetDir(segPtr->dir[i]);
      axes_p->axe_p[i]->SetState(segPtr->dir[i],segPtr->period[i]);
      axes_p->axe_p[i]->SetPeriod(segPtr->period[i],segPtr->prescaler[i],true);   
      axes_p->runData.calCnt[i] = segPtr->calibTicks[i];
      axes_p->runData.pulsesLeft[i] = segPtr->pulses[i];      
    }
    #if TEST_AXE_DET == 1
    printf(" New Segment: totalLength = %d\n",axes_p->runData.actSegment_p->totalLength);
    printf(" V0=%f, VM=%f, VE=%f\n", axes_p->runData.actSegment_p->v0,axes_p->runData.actSegment_p->vM, axes_p->runData.actSegment_p->vE);
    printf("AXE|  Pulses| Period\n");
    for(int i=0;i<NO_OF_AXES;i++)
    {
      printf("%3d|%7d|%7d\n",i,axes_p->runData.actSegment_p->pulses[i],axes_p->runData.actSegment_p->period[i]);
    }
    #endif


    axes_p->runData.actSpeed = segPtr->v0;
    CncAxeProcess_c::actSeqNo = segPtr->seqNo;

    Axe_c::MasterSetSpeed(segPtr->v0);
    DebugSpeed(segPtr->v0,0);
    Axe_c::SyncRun(segPtr->totalLength);

    Axe_c::SpeedRun();
    Axe_c::MasterRun();
   
  }
HAL_GPIO_WritePin(TEST6_PORT,TEST6_PIN,GPIO_PIN_RESET);
}

void CncAxeProcess_c::HandleManualMove( CNC_ManualMove_c* recSig_p)
{
  if(recSig_p->start)
  {

    if(CncStatus_c::StatusOk() == true)
    {
      Axe_c* axe_p = axes_p->axe_p[recSig_p->axe];

      uint32_t period = 100 *  axe_p->GetScale();

      for(int i=0;i<AXE_NOOF;i++)
      {
        axes_p->axe_p[i]->SetDir(DIR_NONE);
        axes_p->axe_p[i]->SetState(DIR_NONE,0);
        axes_p->axe_p[i]->SetPeriod(0,0,true);
      }

      DIR_et dir;
      if(recSig_p->direction > 0)
      {
        dir = DIR_UP;
      }
      else
      {
        dir = DIR_DOWN;
      }

      axes_p->runData.regularMove = MOVETYPE_MANUAL;
      axes_p->runData.state = RUN_RUNNING;

      axes_p->runData.maxV = recSig_p->maxSpeed;
      float dt = 0.001 *SPEED_PERIOD ;
      axes_p->runData.dV = recSig_p->maxAcc * dt;

      axe_p->SetDir(dir);
      axe_p->SetState(dir,period/2);
      axe_p->SetPeriod(period,0,true);

      axes_p->runData.actSpeed = axes_p->runData.dV;
      Axe_c::MasterSetSpeed(axes_p->runData.actSpeed);
      Axe_c::SpeedRun();
      Axe_c::MasterRun();
    }

  }
  else
  {
    if(axes_p->runData.state == RUN_RUNNING)
    {
      axes_p->runData.state = RUN_SLOWING;
    }

    while(axes_p->runData.state != RUN_IDLE)
    {
      vTaskDelay(5);
    }

    for(int i=0;i<NO_OF_AXES;i++)
    {
      MoveProcessor_c::SetPosition(i, pos[i]);
    }

    CncStatus_c::SetStatusBit(STATUS_BIT_MMOVE_END);
  }


}

void SyncTimerCallback(struct __TIM_HandleTypeDef *htim)
{
  HAL_GPIO_WritePin(TEST4_PORT,TEST4_PIN,GPIO_PIN_SET);

  int timers[NO_OF_AXES];
  int syncTimer = AXE_SYNC_Timer->CNT;
  timers[0] = AXE_X_TIMER.Instance->CNT;
  timers[1] = AXE_Y_TIMER.Instance->CNT,
  timers[2] = AXE_Z_TIMER.Instance->CNT,
  timers[3] = AXE_A_TIMER.Instance->CNT;  

  int sum = syncTimer + timers[0] + timers[1] + timers[2] + timers[3];
  bool printDebug = false;
  if(sum > 0)
  {/*
     printDebug = true;

      printf("SYNC = %d, AXES TIM = %d %d %d %d\n",
    syncTimer,
    timers[0],
    timers[1] ,
    timers[2] ,
    timers[3]);

    printf("X: arr=%d, pres=%d, pulses_left = %d\n", AXE_X_Timer->ARR, AXE_X_Timer->PSC, axes_p->runData.pulsesLeft[0]);
*/
  
  }

  #if TEST_SEGLIST == 1
  bool allOk = SaveDebugData(axes_p->runData.actSegment_p);
  if(allOk == false)
  {
    PrintDebugData();
    printDebug = true; 
  }
  #endif

  #if TEST_AXE_PIPELINE == 1
  printf("Real pos = (%d,%d,%d,%d)\n",CncAxeProcess_c::pos[0],CncAxeProcess_c::pos[1],CncAxeProcess_c::pos[2],CncAxeProcess_c::pos[3]);

  #endif
  if(axes_p->runData.regularMove == MOVETYPE_REGULAR)
  {
    Segment_c* segPtr = Segment_c::GetFromListISR();

    for(int i=0;i<NO_OF_AXES;i++)
    {
      if(interruptCheck[i] == false)
      {
        printf("interruptCheck fail idx=%d\n",i);
      }
      interruptCheck[i] = false;

    }

    if(segPtr != nullptr)
    { 
      //printf("Del: Vs=%.2f Ve=%.2f\n",axes_p->runData.actSegment_p->v0,axes_p->runData.actSegment_p->vE);

      //printf("next seg, act speed=%f, VE=%f, VS=%f\n",axes_p->runData.actSpeed,axes_p->runData.actSegment_p->vE,segPtr->v0);

      #if TEST_AXE_SPEED >= 1
      if(( abs(axes_p->runData.actSpeed-axes_p->runData.actSegment_p->vE) > (axes_p->runData.actSegment_p->dV + 0.1)) || ( abs(axes_p->runData.actSpeed-segPtr->v0) > (axes_p->runData.actSegment_p->dV + 0.1)))
      {
        printf("next seg error, act speed=%f, VE=%f, VS=%f\n",axes_p->runData.actSpeed,axes_p->runData.actSegment_p->vE,segPtr->v0);

        Segment_c* prevSegPtr = axes_p->runData.actSegment_p;
        printf("prevSeg: (%d,%d,%d,%d) V0=%f VE=%f VM=%f\n",prevSegPtr->pulses[0],prevSegPtr->pulses[1],prevSegPtr->pulses[2],prevSegPtr->pulses[3],prevSegPtr->v0,prevSegPtr->vE,prevSegPtr->vM);
        printf("prevSeg: (%d,%d,%d,%d) V0=%f VE=%f VM=%f\n",segPtr->pulses[0],segPtr->pulses[1],segPtr->pulses[2],segPtr->pulses[3],segPtr->v0,segPtr->vE,segPtr->vM);
        //CncAxeProcess_c::PrintSpeedDebug();
      }
      #endif

      if(printDebug)
      {
        Segment_c* prevSegPtr = axes_p->runData.actSegment_p;
        printf("prev Seg: (%d,%d,%d,%d) V0=%f VE=%f VM=%f\n",prevSegPtr->pulses[0],prevSegPtr->pulses[1],prevSegPtr->pulses[2],prevSegPtr->pulses[3],prevSegPtr->v0,prevSegPtr->vE,prevSegPtr->vM);
        printf("act  Seg: (%d,%d,%d,%d) V0=%f VE=%f VM=%f\n",segPtr->pulses[0],segPtr->pulses[1],segPtr->pulses[2],segPtr->pulses[3],segPtr->v0,segPtr->vE,segPtr->vM);
        //CncAxeProcess_c::PrintSpeedDebug();

      }

      for(int i=0;i< AXE_NOOF;i++)
      {
        axes_p->axe_p[i]->SetDir(segPtr->dir[i]);
      }
    
      axes_p->runData.actSegment_p->AddToEmptyISR();
      axes_p->runData.actSegment_p = segPtr;
      Axe_c::MasterSetSpeed( axes_p->runData.actSegment_p->v0);
      axes_p->runData.actSpeed = segPtr->v0;
      CncAxeProcess_c::DebugSpeed(segPtr->v0,4);

      if(segPtr->orderCode == MOVE_SET_SPINDLE)
      {
        Axe_c::SetSpindleSpeed(segPtr->value);
      }

      CncAxeProcess_c::actSeqNo = segPtr->seqNo;

      Axe_c::SyncSetPeriod(segPtr->totalLength);

      #if TEST_AXE_SPEED >= 1
      printf("VS = %.2f VE = %.2f\n",segPtr->v0, segPtr->vE);
      #endif

      #if TEST_AXE_DET == 1
      printf(" Cont Segment: totalLength = %d\n",axes_p->runData.actSegment_p->totalLength);
      printf(" V0=%f, VM=%f, VE=%f\n", axes_p->runData.actSegment_p->v0,axes_p->runData.actSegment_p->vM, axes_p->runData.actSegment_p->vE);
      printf("AXE|  Pulses| Period\n");
      for(int i=0;i<NO_OF_AXES;i++)
      {
        printf("%3d|%7d|%7d\n",i,axes_p->runData.actSegment_p->pulses[i],axes_p->runData.actSegment_p->period[i]);
      }
      #endif

      //Axe_c::SpeedReset();

      /*
      printf(" SEG: %d(%d,%d,%d) : (%d,%d,%d) \n",
      axes_p->runData.actSegment_p->totalLength,
      axes_p->runData.actSegment_p->pulses[0],
      axes_p->runData.actSegment_p->pulses[1],
      axes_p->runData.actSegment_p->pulses[2],
      axes_p->runData.actSegment_p->period[0],
      axes_p->runData.actSegment_p->period[1],
      axes_p->runData.actSegment_p->period[2]);
  */



    }
    else 
    {
      Axe_c::MasterStop();
      Axe_c::SpeedStop();



      #if TEST_AXE == 1
      printf("SYNC = %d, AXES TIM = %d %d %d %d\n",
      AXE_SYNC_Timer->CNT,
      AXE_X_Timer->CNT,
      AXE_Y_Timer->CNT,
      AXE_Z_Timer->CNT,
      AXE_A_Timer->CNT);
      #endif
      AXE_SYNC_Timer->CNT = 0;

      for(int i=0;i< AXE_NOOF;i++)
      {
        axes_p->axe_p[i]->SetDir(DIR_UNN);
      }
     /* printf("SEG END\n");*/

      axes_p->runData.actSegment_p = nullptr;
      Axe_c::MasterSetSpeed(Axe_c::GetMinSpeed());
      CncAxeProcess_c::actSeqNo = -1;
    }
  }
  HAL_GPIO_WritePin(TEST4_PORT,TEST4_PIN,GPIO_PIN_RESET);
}


void CalibTimerCallback(struct __TIM_HandleTypeDef *htim)
{
  for(int i=0;i<AXE_NOOF;i++)
  {

    if(axes_p->runData.calCnt[i]> 0)
    {
      axes_p->axe_p[i]->AddCycle();
      axes_p->runData.calCnt[i]--;
    }
  }

}




void SpeedTimerCallback(struct __TIM_HandleTypeDef *htim)
{
//HAL_GPIO_WritePin(TEST5_PORT,TEST5_PIN,GPIO_PIN_SET);
  int seqNo = -1;

  if(axes_p->runData.regularMove == MOVETYPE_REGULAR)
  {

    float S_ToEnd;
    float S_RampDown;

    uint32_t length = Axe_c::GetSyncTimer();
    Segment_c* segment_p = axes_p->runData.actSegment_p;

    S_ToEnd = segment_p->totalLength - length;

    S_ToEnd /= PULSE_MULTIPLIER;

    int  N = (axes_p->runData.actSpeed - segment_p->vE) / segment_p->dV;

    //printf ("N=%d dV=%.2f, V=%.2f, vE=%.2f Send=%0.2f ",segment_p->dV,axes_p->runData.actSpeed,segment_p->vE,N,S_ToEnd);
  /*
    if(segment_p->dV == 0)
    {
      printf("error , segment_p->dV = 0\n");
    }*/
  
    float newSpeed;
    int event = 0;

    if(N >= 0)
    {
      /*act speed is greather than end speed */

      S_RampDown = N* segment_p->dt * (axes_p->runData.actSpeed - 0.5*N*segment_p->dV);

    

      //printf("S_r=%.2f",S_RampDown);



      if(S_RampDown+0.01 >= S_ToEnd)
      {
        if(N == 0)
        {
          newSpeed = segment_p->vE;
          float minSpeed = Axe_c::GetMinSpeed();
          if(newSpeed < minSpeed) { newSpeed = minSpeed; }
          Axe_c::MasterSetSpeed(newSpeed);
          axes_p->runData.actSpeed = newSpeed;
          event = 1;  
        }
        else
        {
          newSpeed = axes_p->runData.actSpeed - segment_p->dV;
          float minSpeed = Axe_c::GetMinSpeed();
          if(newSpeed < minSpeed) { newSpeed = minSpeed; }
          Axe_c::MasterSetSpeed(newSpeed);
          axes_p->runData.actSpeed = newSpeed;
          event = 1;
        }
        //CncAxeProcess_c::DebugSpeed(newSpeed,1);
      }
      else if(axes_p->runData.actSpeed < segment_p->vM)
      {
        N++;
        float speedTmp = axes_p->runData.actSpeed + segment_p->dV;
        S_RampDown = N* segment_p->dt * (speedTmp - 0.5*N*segment_p->dV);

        S_ToEnd = S_ToEnd - (segment_p->dt * speedTmp);

        //printf("Sr2=%.2f, Se2=%.2f",S_RampDown,S_ToEnd);
      
        if(S_RampDown <= S_ToEnd+0.01)
        {
          newSpeed = axes_p->runData.actSpeed + segment_p->dV;
          Axe_c::MasterSetSpeed(newSpeed);
          axes_p->runData.actSpeed = newSpeed;
          event = 2;
          //CncAxeProcess_c::DebugSpeed(newSpeed,2);
        }
      }

    }
    else if(N< 0 )
    {
      newSpeed = axes_p->runData.actSpeed + segment_p->dV;
      Axe_c::MasterSetSpeed(newSpeed);
      axes_p->runData.actSpeed = newSpeed;
      event = 3;


    }

    if(event > 0)
    {
      //CncAxeProcess_c::DebugSpeed(newSpeed,event);
    }
    seqNo = segment_p->seqNo;
  //printf("\n");
  }
  else if((axes_p->runData.regularMove == MOVETYPE_PROBE) || (axes_p->runData.regularMove ==MOVETYPE_MANUAL) || (axes_p->runData.regularMove ==MOVETYPE_BASE))
  {
    if(axes_p->runData.state == RUN_RUNNING)
    {
      if(axes_p->runData.actSpeed < axes_p->runData.maxV)
      { 
        axes_p->runData.actSpeed += axes_p->runData.dV;
        if(axes_p->runData.actSpeed > axes_p->runData.maxV)
        {
          axes_p->runData.actSpeed = axes_p->runData.maxV;
        }
        Axe_c::MasterSetSpeed(axes_p->runData.actSpeed);
      
      }

    }
    else if(axes_p->runData.state == RUN_SLOWING)
    {
      if(axes_p->runData.actSpeed > axes_p->runData.dV)
      {
        axes_p->runData.actSpeed -= axes_p->runData.dV;

        if(axes_p->runData.actSpeed < 0.8*axes_p->runData.dV)
        {
          axes_p->runData.actSpeed = 0.8*axes_p->runData.dV;
        }
        Axe_c::MasterSetSpeed(axes_p->runData.actSpeed);

      }
      else
      {
        /* STOP */
        axes_p->runData.state = RUN_STOPPING;
        //printf("RUN_STOPPING\n");

      }
    }
    else
    {


    }

    if(axes_p->runData.regularMove ==MOVETYPE_BASE)
    {
      seqNo = -2;
    }



  }


//HAL_GPIO_WritePin(TEST5_PORT,TEST5_PIN,GPIO_PIN_RESET);
}

uint32_t TestDirPin[] = {TEST1_PIN,TEST2_PIN,TEST3_PIN};
GPIO_TypeDef* TestDirPort[] = {TEST1_PORT,TEST2_PORT,TEST3_PORT};  

void AxeSoftPosCallback(struct __TIM_HandleTypeDef *htim)
{
  int idx = 0;

  for(idx=0;idx<NO_OF_AXES;idx++)
  {
    if(htim == axes_p->axe_p[idx]->GetHtim())
    {
      break;
    }
  }




  //HAL_GPIO_WritePin(TestDirPort[idx],TestDirPin[idx],GPIO_PIN_SET);
  DIR_et dir;
  int scale;

  dir = axes_p->axe_p[idx]->GetActDir(); 
  scale = axes_p->axe_p[idx]->GetScale();

  bool count = true;

  if(axes_p->runData.regularMove == MOVETYPE_REGULAR) 
  {

    Segment_c* actSegment_p = axes_p->runData.actSegment_p;
    
    if(axes_p->runData.calCnt[idx]> 0)
    {
    HAL_GPIO_WritePin(TestDirPort[idx],TestDirPin[idx],GPIO_PIN_SET);
      axes_p->axe_p[idx]->SetPeriod(actSegment_p->period[idx]+1,actSegment_p->prescaler[idx],false); /* todo +1 */
      axes_p->runData.calCnt[idx]--;

      //printf("axe %d P\n",idx);
    }
    else
    {
      axes_p->axe_p[idx]->SetPeriod(actSegment_p->period[idx],actSegment_p->prescaler[idx],false);
      //printf("axe %d N\n",idx);
    }


    axes_p->runData.pulsesLeft[idx] --;

    if(axes_p->runData.pulsesLeft[idx] == 0)
    {
    //HAL_GPIO_WritePin(TestDirPort[idx],TestDirPin[idx],GPIO_PIN_SET);
      Segment_c* nextSeg =Segment_c::ReadFirstISR();
      if(nextSeg != nullptr) 
      {
        axes_p->runData.pulsesLeft[idx] = nextSeg->pulses[idx];
        axes_p->runData.calCnt[idx] = nextSeg->calibTicks[idx];        
        
        axes_p->axe_p[idx]->SetPeriod(nextSeg->period[idx],nextSeg->prescaler[idx],false);
        axes_p->axe_p[idx]->SetState(nextSeg->dir[idx],nextSeg->period[idx]);
        //printf("axe %d update Time=%d\n",idx,AXE_SYNC_Timer->CNT);
      }
      else
      {
        axes_p->runData.pulsesLeft[idx] = 0;
        axes_p->runData.calCnt[idx] = 0;
        axes_p->axe_p[idx]->SetState(DIR_NONE,0);
        //axes_p->axe_p[idx]->SetPeriod(0,0,false);

        //printf("axe %d end Time=%d\n",idx,AXE_SYNC_Timer->CNT);
      }
      interruptCheck[idx] = true;
    }


  }
  if((axes_p->runData.regularMove == MOVETYPE_PROBE) || (axes_p->runData.regularMove == MOVETYPE_MANUAL) || (axes_p->runData.regularMove == MOVETYPE_BASE))
  {
  
    if(axes_p->runData.state == RUN_STOPPING)
    {
      count = false;
      Axe_c::MasterStop();
      Axe_c::SpeedStop();
      axes_p->runData.state = RUN_IDLE;
      //printf("RUN_IDLE\n");
    } 
  } 

  if(count == true)
  {
    if(dir == DIR_UP)
    {
      CncAxeProcess_c::pos[idx]+=scale;
    }
    else if(dir == DIR_DOWN)
    {
      CncAxeProcess_c::pos[idx]-=scale;
    }
  }

  HAL_GPIO_WritePin(TestDirPort[idx],TestDirPin[idx],GPIO_PIN_RESET);
}



CncAxes_c::CncAxes_c(void)
{
  for(int i =0;i<AXE_NOOF;i++)
  {
    axe_p[i] = new Axe_c(i);
  }
}

void CncAxes_c::Init()
{


  for(int i =0;i<AXE_NOOF;i++)
  {
    axe_p[i]->Init();
  }
  Axe_c::MasterTimInit();
  Axe_c::SyncTimInit();
  Axe_c::SpeedTimInit();
  Axe_c::InitSpindlePhy();

   for(int i =0;i<AXE_NOOF;i++)
  { 
    axe_p[i]->Run();
  }

}


void BreakMove(void )
{



}



void CncAxes_c::CalcSegment(Segment_c* segment_p,iVector3D* vector_p, int aMove,float v0, float vE, float vMax, float aMax)
{
  uint32_t pulses[AXE_NOOF];
  uint32_t period[AXE_NOOF];
  uint32_t prescaler[AXE_NOOF];
  bool enabled[AXE_NOOF];

  int vectorArr[AXE_NOOF];

  vectorArr[0] = vector_p->x;
  vectorArr[1] = vector_p->y;
  vectorArr[2] = vector_p->z;
  vectorArr[3] = aMove;

  for(int i=0;i< AXE_NOOF ; i++)
  {
    if(vectorArr[i] < 0) 
    { 
      pulses[i] = -vectorArr[i]; 
      segment_p->dir[i] = DIR_DOWN; 
    } 
    else if(vectorArr[i] > 0) 
    {
      pulses[i] = vectorArr[i]; 
      segment_p->dir[i] = DIR_UP;
    }
    else
    {
      pulses[i] = 0; 
      segment_p->dir[i] = DIR_NONE;
    }
  }

  for(int i=0;i< AXE_NOOF ; i++)
  {
    pulses[i] =  pulses[i] / axe_p[i]->GetScale();
  }

  for(int i=0;i< AXE_NOOF ; i++)
  {
    if(pulses[i] == 0)
    {
      enabled[i] = false;
      pulses[i] = 2;
    }
    else
    {
      enabled[i] = true;
    }
  }

  Vector3D vf(vector_p->x,vector_p->y,vector_p->z);
  vf *= 0.001; /* iVector is given in [um], change to [mm] ; */
  float length = vf.Length();
  if(length == 0)
  {
    length = 0.001 * abs(aMove);
  }

  uint32_t totalCycles = PULSE_MULTIPLIER * length ;

  uint32_t largestPeriod = 0;

  uint32_t periodTest[AXE_NOOF];

  int largestDivider = 0;

  /* Calc period and prescaler */
  int divider[AXE_NOOF];

  for(int i=0;i<AXE_NOOF;i++)
  {
    period[i] = totalCycles / pulses[i];
    divider[i] = 0;

    while(period[i] > 0xFF00)
    {
      divider[i]++;
      period[i] >>= 1;
    }
    prescaler[i] = (1<<divider[i]) - 1;
    if(largestDivider < divider[i])
    {
      largestDivider = divider[i];
    }
  }

  /* Roundup total cycles to match with max divider */

  int k = largestDivider;

  uint32_t totalCyclesTmp = totalCycles;
  uint32_t totalCyclesTest = totalCycles;

  if(k>0)
  {
    totalCyclesTmp>>=k;
    totalCyclesTmp<<=k;
    if(totalCyclesTmp != totalCycles)
    {
      totalCycles>>=k;
      totalCycles++;
      totalCycles<<=k;
    }
  }

  /* calc period */
 
  for(int i=0;i<AXE_NOOF;i++)
  {
    period[i] = totalCycles / pulses[i];
    period[i] >>= divider[i];
  }



  /*calc cycles to add */

  uint32_t cyclesToAdd[AXE_NOOF];

  for(int i=0;i<AXE_NOOF;i++)
  {
    cyclesToAdd[i] = totalCycles - (pulses[i] * (prescaler[i]+1) * period[i]);
    cyclesToAdd[i] >>= divider[i];

  }

  uint32_t calibPeriod = 0;

  for(int i=0;i<AXE_NOOF;i++)
  {
    segment_p->period[i] = period[i];
    segment_p->calibTicks[i] = cyclesToAdd[i];
    segment_p->prescaler[i] = prescaler[i];
    segment_p->pulses[i] = pulses[i];
  }
  segment_p->periodCalib = calibPeriod;
  segment_p->totalLength = totalCycles;


  /*self check */
  #if TEST_AXE > 0
  for(int  i=0;i<AXE_NOOF;i++)
  {
    int axeCycles = ((prescaler[i]+1) * period[i] * pulses[i]) + cyclesToAdd[i]*(prescaler[i]+1);

    if(axeCycles != totalCycles)
    {
      printf("error, largest=%d totalCycles=%d\n",largestDivider, totalCyclesTest);
      printf("Org period = %d %d %d %d\n",periodTest[0],periodTest[1],periodTest[2],periodTest[3]);

    }
  }
  #endif

  if(aMax == 0)
  {

    segment_p->v0 = v0;
    segment_p->vE = vE;
  }
  else
  {
    CalcSpeeds(segment_p,length,v0,vE,vMax,aMax);
  }

  segment_p->lx = vector_p->x;
  segment_p->ly = vector_p->y;
  segment_p->lz = vector_p->z;

  #if TEST_AXE == 1
  printf("Move: total cycles = %d\n",totalCycles);
  printf("AXE|   Move| Period|  Presc|  Calib|  Pulses\n");
  for(int i=0;i<NO_OF_AXES;i++)
  {
    printf("%3d|%7d|%7d|%7d|%7d|%7d\n",i,vectorArr[i],segment_p->period[i],segment_p->prescaler[i],segment_p->calibTicks[i],segment_p->pulses[i]);
  }
  #endif


}

void CncAxes_c::CalcSpeeds(Segment_c* segment_p,float length, float v0, float vE, float vMax, float aMax)
{
  segment_p->v0 = v0;
  segment_p->vE = vE;
  segment_p->vM = vMax;
  segment_p->dt =  0.001 *SPEED_PERIOD ;
  segment_p->dV = aMax * segment_p->dt;
}


