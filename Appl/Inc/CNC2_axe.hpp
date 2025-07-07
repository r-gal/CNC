#ifndef CNC2_AXE_H
#define CNC2_AXE_H

#include "SignalList.hpp"
#include "CommonDef.hpp"
#include "CNC2_dataDef.hpp"

class Axe_c
{
  const int idx;

  

  DIR_et actDir;

  uint8_t limState;

  bool ena;
  bool axeDir;
  int scale;
  LIMITER_TYPE_et limType;
  LIMITER_MODE_et limMode;

  static int estopMode;
  static int probeMode;

  static float minSpeed;

  float maxAcc;
  float maxSpeed;

  public:

  TIM_HandleTypeDef* GetHtim(void);

  static float actSpeed;

  float GetMaxSpeed(void) { return maxSpeed; }
  float GetMaxAcc(void) { return maxAcc; }


  int GetScale(void) { return  scale; }
    
   Axe_c(int idx);

   void Init(void);
   void GpioInit(void);
   void SetDir(DIR_et dir);
   DIR_et GetActDir(void);
   void SetState(DIR_et dir, uint32_t period);


   void Run(void);
   void SetPeriod(uint16_t period, uint16_t prescaler,bool force);
   void Reset(void);
   
   void AddCycle(void);

   uint8_t GetLimState(void);
   bool LimActive(void) { return (limMode != LIM_NONE); }

   static uint8_t GetProbeState(void);
   static bool ProbeActive(void) { return true; }

   static bool GetEStopState(void);

   static void SetEstopConfig(int mode);
   static void SetProbeConfig(int mode);
   static void SetMiscConfig(float minSpeed_) { minSpeed = minSpeed_; }
   static float GetMinSpeed(void) { return minSpeed; }


   static void MasterTimInit(void);
   static void SyncTimInit(void);
   static void SpeedTimInit(void);
   static void MasterRun(void);
   static void MasterStop(void);
   static void MasterSetSpeed(float speed);
   static void SyncRun(uint32_t period);
   static void SyncSetPeriod(uint32_t period);
   static void SyncStop(void);
   static uint32_t GetSyncTimer(void);
   static void SpeedRun(void);
   static void SpeedStop(void);
   static void SpeedReset(void);

   static void InitSpindlePhy(void);
   static void SetSpindleSpeed(int speed);

   void SetConfig(bool ena_,bool dir_, int scale_, LIMITER_MODE_et limMode_, LIMITER_TYPE_et limType_, float maxSpeed_, float maxAcc_);

};

#ifdef __cplusplus
 extern "C" {
#endif

void TIM5_IRQHandler(void);
void SyncTimerCallback(struct __TIM_HandleTypeDef *htim);

void TIM23_IRQHandler(void);

void TIM16_IRQHandler(void);
void SpeedTimerCallback(struct __TIM_HandleTypeDef *htim);

void TIM2_IRQHandler(void);
void TIM5_IRQHandler(void);
void TIM15_IRQHandler(void);
void TIM24_IRQHandler(void);

void AxeSoftPosCallback(struct __TIM_HandleTypeDef *htim);


#ifdef __cplusplus
}
#endif


#endif