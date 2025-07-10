 #include <stdio.h>
#include <stdlib.h>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "main.h"


#include "CNC2_axe.hpp"

extern TIM_HandleTypeDef AXE_X_TIMER;
extern TIM_HandleTypeDef AXE_Y_TIMER;
extern TIM_HandleTypeDef AXE_Z_TIMER;
extern TIM_HandleTypeDef AXE_A_TIMER;
extern TIM_HandleTypeDef AXE_B_TIMER;

extern TIM_HandleTypeDef MASTER_TIMER;
extern TIM_HandleTypeDef SPEED_TIMER;
extern TIM_HandleTypeDef SYNC_TIMER;
extern TIM_HandleTypeDef SPINDLE_TIMER;


TIM_HandleTypeDef* TimInstance[] = {&AXE_X_TIMER,&AXE_Y_TIMER,&AXE_Z_TIMER,&AXE_A_TIMER,&AXE_B_TIMER};
/*uint32_t TimChannel[] = {AXE_X_Timer_ch,AXE_Y_Timer_ch,AXE_Z_Timer_ch,AXE_A_Timer_ch,AXE_B_Timer_ch};*/

uint32_t DirPin[] = {dirX_Pin,dirY_Pin,dirZ_Pin,dirA_Pin,dirB_Pin};
GPIO_TypeDef* DirPort[] = {dirX_GPIO_Port,dirY_GPIO_Port,dirZ_GPIO_Port,dirA_GPIO_Port,dirB_GPIO_Port}; 

uint32_t LimPin[] = {limX_Pin,limY_Pin,limZ_Pin,limA_Pin,limB_Pin};
GPIO_TypeDef* LimPort[] = {limX_GPIO_Port,limY_GPIO_Port,limZ_GPIO_Port,limA_GPIO_Port,limB_GPIO_Port}; 
/*
uint32_t Lim2Pin[] = {AXE_X_LIMIT2_Pin,AXE_Y_LIMIT2_Pin,AXE_Z_LIMIT2_Pin};
GPIO_TypeDef* Lim2Port[] = {AXE_X_LIMIT2_Port,AXE_Y_LIMIT2_Port,AXE_Z_LIMIT2_Port};
*/
int Axe_c::estopMode = 0;
int Axe_c::probeMode = 0;
float Axe_c::minSpeed = 0.5;

void Axe_c::SetConfig(bool ena_,bool dir_, int scale_, LIMITER_MODE_et limMode_, LIMITER_TYPE_et limType_, float maxSpeed_, float maxAcc_)
{
  ena = ena_;
  axeDir = dir_;
  scale = scale_;
  limMode = limMode_;
  limType = limType_;
  maxSpeed = maxSpeed_;
  maxAcc= maxAcc_;

}

Axe_c::Axe_c(int idx) : idx(idx)
{
  scale = 10;
  ena = false;
  axeDir = false;
  limMode = LIM_NONE;
  limType = LIM_TYPE_NO;
  actDir = DIR_UNN;

  maxAcc = 100;
  maxSpeed = 5;

}

TIM_HandleTypeDef* Axe_c::GetHtim(void) 
{ 
  return TimInstance[idx];
}



bool Axe_c::GetEStopState(void)
{

  bool state = (GPIO_PIN_SET == HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port,EMERGENCY_Pin));
  if(estopMode == 1)
  {
    state = !state;
  }
  return state;
}

uint8_t Axe_c::GetProbeState(void)
{
  bool state = (GPIO_PIN_SET == HAL_GPIO_ReadPin(PROBE_GPIO_Port,PROBE_Pin));
  uint8_t probeState = 0;
  if((probeMode == LIM_TYPE_NO)^ (state))
  {
    probeState |= 0x01;
  }
  return probeState;
}

uint8_t Axe_c::GetLimState(void)
{

  switch(limMode)
  {
    case LIM_NONE:
      return 0;
    case LIM_ONE:
      {
        bool state = (GPIO_PIN_SET == HAL_GPIO_ReadPin(LimPort[idx],LimPin[idx]));
        if((limType == LIM_TYPE_NO)^ (state))
        {
          switch(actDir)
          {
            case DIR_DOWN:
              if(limState == 0)
              {
                limState = 0x01; 
                printf("Set state 1, axe%d\n",idx);
              }
              break;
            case DIR_UP:
              if(limState == 0)
              {
                limState = 0x02;
                printf("Set state 2, axe%d\n",idx);
              }
              break;
            case DIR_UNN:
              limState = 0x03; break;
          }        
        }
        else
        {
          if(limState != 0)
          {
            printf("Set state 0, axe%d\n",idx);
          }
          limState = 0;
        }
      }
      return limState;

    case LIM_DUAL:
      {
   /*     bool state = (GPIO_PIN_SET == HAL_GPIO_ReadPin(LimPort[idx],LimPin[idx]));
        bool state2 = (GPIO_PIN_SET == HAL_GPIO_ReadPin(Lim2Port[idx],Lim2Pin[idx]));
        limState = 0;
        if(limType == LIM_TYPE_NO)
        {
          if(state == false) { limState |= 0x01; }
          if(state2 == false) { limState |= 0x02; }
        }
        else
        {
          if(state == true) { limState |= 0x01; }
          if(state2 == true) { limState |= 0x02; }
        }*/
      }
      return limState;
      default :
      limState = 0;
      return 0;
  }
  
}

void Axe_c::SetDir(DIR_et dir)
{

  if(dir == DIR_NONE)
  {
    //TIM_CCxChannelCmd(TimInstance[idx],TimChannel[idx],TIM_CCx_DISABLE);

    //TimInstance[idx]->CCR1 = 0xFFFF;

  }
  else
  {
    //TIM_CCxChannelCmd(TimInstance[idx],TimChannel[idx],TIM_CCx_ENABLE);
    //uint16_t ccr = htim.Instance->ARR /2;
    //TimInstance[idx]->CCR1 = ccr;
    
    if((dir == DIR_UP) ^ axeDir)
    {
      HAL_GPIO_WritePin(DirPort[idx],DirPin[idx],GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(DirPort[idx],DirPin[idx],GPIO_PIN_RESET);
    }
  }
  actDir = dir;

}

void Axe_c::SetState(DIR_et dir, uint32_t period)
{
  if(dir == DIR_NONE)
  {
    //TIM_CCxChannelCmd(TimInstance[idx],TimChannel[idx],TIM_CCx_DISABLE);

    TimInstance[idx]->Instance->CCR1 = 0xFFFF;

  }
  else
  {
    //TIM_CCxChannelCmd(TimInstance[idx],TimChannel[idx],TIM_CCx_ENABLE);
    uint16_t ccr =  period /2;
    TimInstance[idx]->Instance->CCR1 = ccr;
    
  }
  //actDir = dir;

}

DIR_et Axe_c::GetActDir(void)
{
  return actDir;
}


void Axe_c::Run(void)
{
  TIM_HandleTypeDef* htim_p = GetHtim();
  HAL_TIM_PWM_Start(htim_p,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(htim_p,TIM_CHANNEL_2);
}


void Axe_c::SetPeriod(uint16_t period, uint16_t prescaler, bool force)
{
  TIM_HandleTypeDef* htim_p = GetHtim();
  //htim.Instance->CNT = 0;
  if(period > 0)
  {
    period--;
  }
  htim_p->Instance->PSC = prescaler;
  htim_p->Instance->ARR = period;
  if(actDir != DIR_NONE)
  {
    //uint16_t ccr = htim_p->Instance->ARR - 10;
    uint16_t ccr = htim_p->Instance->ARR/2;
    TimInstance[idx]->Instance->CCR1 = ccr;
  }
  if(force)
  {
    htim_p->Instance->EGR = 1; /* UG generation */
  }
}

void Axe_c::Reset(void)
{
 TIM_HandleTypeDef* htim_p = GetHtim();
 htim_p->Instance->CNT = 0;
}


void Axe_c::AddCycle(void)
{
  TIM_HandleTypeDef* htim_p = GetHtim();
  htim_p->Instance->CNT++;
}

void Axe_c::MasterRun(void)
{
  for(int i=0;i<NO_OF_AXES;i++)
  {
    TimInstance[i]->Instance->CNT = 0;
  }
 //AXE_SYNC_Timer->CNT = 0;

  HAL_TIM_Base_Start(&MASTER_TIMER);

}

void Axe_c::MasterStop(void)
{
  HAL_TIM_Base_Stop(&MASTER_TIMER);  
}

float Axe_c::actSpeed = 2;

void Axe_c::MasterSetSpeed(float speed)
{
  #if TEST_AXE_SPEED == 2
  printf("Set speed = %f->%f\n",actSpeed,speed);
  #endif

  actSpeed = speed;
  uint32_t periodInt = 0;


  if( speed > 0)
  {
    float period = F_TIM_MASTER / (speed * 100000);
    periodInt = (uint32_t) period;
    if(periodInt == 0 )
    {
      periodInt = 1;
    }
    else if(periodInt > 1 ) periodInt--;
  }
  
  AXE_MASTER_Timer->ARR = periodInt;
  //printf("Set speed %d\n",periodInt);
}


void Axe_c::SyncRun(uint32_t period)
{
  SYNC_TIMER.Instance->CNT = 0;
  SYNC_TIMER.Instance->ARR = period-1;
  __HAL_TIM_CLEAR_IT(&SYNC_TIMER, TIM_IT_UPDATE);
  //HAL_TIM_Base_Start(&SYNC_TIMER);

  #if TEST_SYNC_TIMER == 2
    HAL_TIM_OC_Start_IT(&SYNC_TIMER,TIM_CHANNEL_2);
  #else
    HAL_TIM_Base_Start_IT(&SYNC_TIMER);
  #endif
  TIM_CCxChannelCmd(TIM23, TIM_CHANNEL_2, TIM_CCx_ENABLE);
  
  //HAL_TIM_Base_Start_IT(&SYNC_TIMER);
  
}

void Axe_c::SyncSetPeriod(uint32_t period)
{
  SYNC_TIMER.Instance->ARR = period-1;  
}

void Axe_c::SyncStop(void)
{

  //HAL_TIM_Base_Start(&SYNC_TIMER);

  #if TEST_SYNC_TIMER ==2
  HAL_TIM_OC_Stop_IT(&SYNC_TIMER,TIM_CHANNEL_2);
  #else
  HAL_TIM_Base_Stop_IT(&SYNC_TIMER);
  #endif
 
  
}

uint32_t Axe_c::GetSyncTimer(void)
{
  return AXE_SYNC_Timer->CNT;
}



void Axe_c::SpeedRun(void)
{
  HAL_TIM_Base_Start_IT(&SPEED_TIMER);
  SPEED_TIMER.Instance->CNT = 0; /*clear counter and trigger interrupt at segment start */
}

void Axe_c::SpeedReset(void)
{
  SPEED_TIMER.Instance->CNT = 0; /*clear counter and trigger interrupt at segment start */
}

void Axe_c::SpeedStop(void)
{
  HAL_TIM_Base_Stop_IT(&SPEED_TIMER);
}

void Axe_c::SetSpindleSpeed(int speed)
{

  if (speed > 0)
  {
    HAL_GPIO_WritePin(MotorEna_GPIO_Port,MotorEna_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MotorDir_GPIO_Port,MotorDir_Pin,GPIO_PIN_SET);
    SPINDLE_TIMER.Instance->CCR1 = speed;
  }
  else if (speed < 0)
  {
    HAL_GPIO_WritePin(MotorEna_GPIO_Port,MotorEna_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(MotorDir_GPIO_Port,MotorDir_Pin,GPIO_PIN_RESET);
    SPINDLE_TIMER.Instance->CCR1 = -speed;
  }
  else
  {
    HAL_GPIO_WritePin(MotorEna_GPIO_Port,MotorEna_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(MotorDir_GPIO_Port,MotorDir_Pin,GPIO_PIN_SET);
    SPINDLE_TIMER.Instance->CCR1 = 0;
  }

  

}

void Axe_c::InitSpindlePhy(void)
{

  HAL_TIM_PWM_Start(&SPINDLE_TIMER,TIM_CHANNEL_1);

}

 void Axe_c::SetEstopConfig(int mode)
 {
    estopMode = mode;

 }
 void Axe_c::SetProbeConfig(int mode)
 {
   probeMode = mode;


 }

 void Axe_c::MasterTimInit(void)
 {

 }
 void Axe_c::SyncTimInit(void)
 {
    SYNC_TIMER.PeriodElapsedCallback = SyncTimerCallback;
 }
 void Axe_c::SpeedTimInit(void)
 {
   SPEED_TIMER.PeriodElapsedCallback = SpeedTimerCallback;
 }
  void Axe_c::Init(void)
 {
   TimInstance[idx]->OC_DelayElapsedCallback = AxeSoftPosCallback;
 }


#ifdef __cplusplus
 extern "C" {
#endif

//void TIM23_IRQHandler(void)
//{
// // printf("S");
//   __HAL_TIM_CLEAR_IT(&SYNC_TIMER, TIM_IT_UPDATE);
//    SyncTimerCallback(&SYNC_TIMER);
// //   printf("s");
//}


//void TIM16_IRQHandler(void)
//{
//  //printf("F");
//  //  __HAL_TIM_CLEAR_IT(&Axe_c::htimSpeed, TIM_IT_UPDATE);
// // SpeedTimerCallback(&Axe_c::htimSpeed);
//   // printf("f");
//}


//void TIM2_IRQHandler(void)
//{
// // printf("X");
//  TIM2->SR =  ~(TIM_IT_CC2);
//  AxeSoftPosCallback(0);
// //   printf("x");
//}


//void TIM5_IRQHandler(void)
//{
// // printf("Y");
//  TIM5->SR =  ~(TIM_IT_CC2);
//  AxeSoftPosCallback(1);
// //   printf("y");
//}


//void TIM15_IRQHandler(void)
//{
// // printf("Z");
//  TIM15->SR =  ~(TIM_IT_CC2);
//  AxeSoftPosCallback(2);
// //   printf("z");
//}

//void TIM24_IRQHandler(void)
//{
// // printf("A");
//  TIM24->SR =  ~(TIM_IT_CC2);
//  AxeSoftPosCallback(3);
//   // printf("a");
//}




#ifdef __cplusplus
}
#endif