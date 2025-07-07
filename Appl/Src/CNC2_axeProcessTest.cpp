#include <stdio.h>
#include <stdlib.h>
#include <cstring>

#define TEST_AXE 
#include "CNC2_axeProcess.hpp"

//void CncAxeProcess_c::CalcSegment(Segment_st* segment,VectorInt_st* vector_p,float speedAtEnd, float maxSpeed, float maxAcc);

void PrintSegment(Segment_st* segment_p)
{

  printf("SEGMENT: T=%d (%d,%d,%d), C=%d (%d,%d,%d)\n", 
  segment_p->totalLength,
  segment_p->period[0],
  segment_p->period[1],
  segment_p->period[2],
  segment_p->periodCalib,
  segment_p->calibTicks[0],
  segment_p->calibTicks[1],
  segment_p->calibTicks[2]);

/*
  printf("S=%.2f V0=%.2f,VE=%.2f, Vmax=%.2f, Amax=%.2f, S0=%.2f S1=%.2f rVMax=%.2f\n",length,v0,vE,vMax,aMax,S0,S1, vMaxReal);
  printf(" S0=%.2f, S1=%.2f t0=%.2f, t1=%.2f, t2=%.2f\n",S0,S1,t0,t1,t2);*/
}

void TestSegment(Segment_st* segment_p)
{
  float dT = 0.1;
  float Stest = 0;



  for(float t=0; t< segment_p->tTotal; t += dT)
  {
    float Va = 0;
    if(t < segment_p->t1)
    {
      Va = segment_p->v0 + segment_p->a * t;
    }
    else if(t> segment_p->t2)
    {
      Va = segment_p->vM - segment_p->a * (t - segment_p->t2);
    }
    else
    {
      Va = segment_p->vM;
    }

    float Vb = 0;
    float tb = t+dT;
    if(tb < segment_p->t1)
    {
      Vb = segment_p->v0 + segment_p->a * tb;
    }
    else if(tb > segment_p->tTotal)
    {
      Vb = segment_p->vE;
    }
    else if(tb> segment_p->t2)
    {
      Vb = segment_p->vM - segment_p->a * (tb - segment_p->t2);
    }
    else
    {
      Vb = segment_p->vM;
    }

    float Vset = 0.5*(Va+Vb);

    float dS = dT * Vset;
    Stest += dS;
    

    printf(" V(%.2f) = %.2f, dS =%.2f S=%.2f\n",t,Vset,dS,Stest);
  }
  


}


int CNC2_AxeProcessTest(void)
{

  printf("test\n");

  CncAxes_c cncAxes;

  Segment_st segment;
/*  iVector3D vector(1000,2000,3000);

  cncAxes.CalcSegment(&segment,&vector,10.0,10.0,20.0);

  PrintSegment(&segment);

  vector = iVector3D(100,100,100);
  cncAxes.CalcSegment(&segment,&vector,10.0,10.0,20.0);
  PrintSegment(&segment);

  vector = iVector3D(100,0,0);
  cncAxes.CalcSegment(&segment,&vector,10.0,10.0,20.0);
  PrintSegment(&segment);  

  vector = iVector3D(100070,10090,12340);
  cncAxes.CalcSegment(&segment,&vector,10.0,10.0,20.0);
  PrintSegment(&segment);  

  cncAxes.CalcSpeeds(&segment,10,0,0,2,5);
  TestSegment(&segment);
  cncAxes.CalcSpeeds(&segment,10,0,0,10,5);
  TestSegment(&segment);
  cncAxes.CalcSpeeds(&segment,10,0,0,20,5);
  TestSegment(&segment);

  cncAxes.CalcSpeeds(&segment,10,2,0,4,5);
  TestSegment(&segment);
  cncAxes.CalcSpeeds(&segment,10,0,2,4,5);
  TestSegment(&segment);
  cncAxes.CalcSpeeds(&segment,10,2,2,4,5);
  TestSegment(&segment);

  cncAxes.CalcSpeeds(&segment,10,4,0,4,5);
  TestSegment(&segment);
  cncAxes.CalcSpeeds(&segment,10,0,4,4,5);
  TestSegment(&segment);
  cncAxes.CalcSpeeds(&segment,10,4,4,4,5);
  TestSegment(&segment);

  cncAxes.CalcSpeeds(&segment,16,0,8,10,2);
  TestSegment(&segment);
  cncAxes.CalcSpeeds(&segment,16,8,0,10,2);
  TestSegment(&segment);*/

  iVector3D vector = iVector3D(0,0,1000);

  cncAxes.CalcSegment(&segment,&vector,0,0,5,5);  
  TestSegment(&segment);

  return 1;


}