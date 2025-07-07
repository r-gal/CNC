#ifndef CNC2_SEGMENT_H
#define CNC2_SEGMENT_H

#include "SignalList.hpp"
#include "CommonDef.hpp"


#include "RngClass.hpp"
#include "TimeClass.hpp"

#include "CNC2_axe.hpp"
#include "Vector.hpp"

#include "CNC2_dataDef.hpp"

#define USE_SAVE_QUEUES 1

class Segment_c
{

  Segment_c* next;

  #if USE_SAVE_QUEUES == 1 
  static QueueHandle_t queue;
  static QueueHandle_t emptyQueue;
  #else
  static Segment_c* first;
  static Segment_c* last;
  static Segment_c* emptyList;   
  #endif

  public:

  void AddToEmpty(void);
  void AddToEmptyISR(void);
  static Segment_c* GetEmpty(void);

  bool AddToList(void);
  static Segment_c* GetFromList(void);
  static Segment_c* GetFromListISR(void);
  
  static Segment_c* ReadFirstISR(void);

  static void ResetList(void);

  static void InitList(void);

 
  Segment_c(void)
  {
    next = nullptr;
  }

  /* vector data */
  uint32_t totalLength;
  uint32_t periodCalib;
  uint32_t pulses[AXE_NOOF];
  uint16_t period[AXE_NOOF];
  uint32_t calibTicks[AXE_NOOF];
  uint16_t  prescaler[AXE_NOOF];
  
  
  int16_t value;
  DIR_et dir[AXE_NOOF];
  uint8_t orderCode;
  

  /*speed data */
  float v0;
  float vM;
  float vE;
  float dV;
  float dt;

  int seqNo;

  int lx,ly,lz;

  /* debug data */
  uint32_t endPos[AXE_NOOF];
  int segmentNr;
  
};


#endif