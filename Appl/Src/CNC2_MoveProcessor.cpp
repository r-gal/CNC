 #include <stdio.h>
#include <stdlib.h>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "math.h"


#include "CNC2_MoveProcessor.hpp"
#include "CNC2_axe.hpp"


int MoveProcessor_c::position[NO_OF_AXES] = {0};



void MoveProcessor_c::GetRampDistances(float* s0,float* s1, float* s2, float* vMaxReal,     float length, float v0, float vE, float Vmax, float aMax)
{
    float sCheck = abs((vE*vE - v0*v0) / (aMax * 2));

    if (sCheck + MIN_LENGTH > length)
    {
        *s0 = length;
        *s1 = 0;
        *s2 = 0;

        *vMaxReal = Vmax;
    }
    else
    {
        *s0 = (Vmax*Vmax - v0*v0) / (aMax * 2);
        *s2 = (Vmax*Vmax - vE*vE) / (aMax * 2);

        *s1 = length - (*s0 + *s2);



        if (*s1 < 0)
        {
            *s1 = -0.5 * *s1;
            *s0 -= *s1;
            *s2 = length - *s0;
            *s1 = 0;

            *vMaxReal = sqrtf(v0*v0 + (2 * aMax * *s0));

            

            /*segment 0: length = S0, V= moveData.startSpeed -> vMaxReal */
            /*segment 1: length = S2, V=  vMaxReal -> moveData.endSpeed */
        }
        else
        {
          *vMaxReal = Vmax;
        }

    }

}

void MoveProcessor_c::InitDwel(MoveProcData_st* runData_p, MoveData_st*  moveSig_p)
{
  runData_p->moveType = moveSig_p->moveType;
  runData_p->delay = moveSig_p->delay;
}


void MoveProcessor_c::InitLine(MoveProcData_st* runData_p, CNC_moveSig_c* moveSig_p, int maxSegLength)
{
  float minSpeed = Axe_c::GetMinSpeed();

  runData_p->startSpeed = moveSig_p->speedStart;
  runData_p->endSpeed = moveSig_p->speedEnd;
  runData_p->vMax = moveSig_p->maxSpeed;

  if (runData_p->startSpeed < minSpeed) {runData_p->startSpeed = minSpeed;}
  if (runData_p->endSpeed < minSpeed) {runData_p->endSpeed = minSpeed;}
  if (runData_p->vMax < minSpeed) {runData_p->vMax = minSpeed;}

  runData_p->rEndPoint = iPoint3D(moveSig_p->x,moveSig_p->y,moveSig_p->z);
  runData_p->endApos = moveSig_p->a;
  runData_p->aMax = moveSig_p->maxAcceleration;
  runData_p->moveType = moveSig_p->moveType;
  runData_p->maxSegLen = maxSegLength;

  bool splitNeeded = false;

  int move[NO_OF_AXES];

  move[0] = runData_p->rEndPoint.x - position[0];
  move[1] = runData_p->rEndPoint.y - position[1];
  move[2] = runData_p->rEndPoint.z - position[2]; 
  move[3] = runData_p->endApos - position[3];

  if(runData_p->maxSegLen > 0)
  {
    int moveX =  move[0];
    int moveY =  move[1];
    int lengthSqr = moveX *moveX + moveY * moveY;

    if( runData_p->maxSegLen *runData_p->maxSegLen < lengthSqr)
    {
      splitNeeded = true;
    }
  }

  if(splitNeeded)
  { 
   

    float x = move[0];
    float y = move[1];
    float z = move[2];

    float maxLen = runData_p->maxSegLen;

    float length = sqrtf(x*x + y*y + z+z);
    float lengthPlane = sqrtf(x*x + y*y);

    runData_p->totalSteps = lengthPlane / maxLen;
    runData_p->totalSteps++;
    runData_p->progress = 0;

    for(int i = 0;i<NO_OF_AXES;i++)
    {
      runData_p->step[i] = move[i] / runData_p->totalSteps;
    }


    float s0,s1,s2,vMaxReal;
    GetRampDistances(&s0,&s1,&s2,&vMaxReal,length,moveSig_p->speedStart,moveSig_p->speedEnd,moveSig_p->maxSpeed,moveSig_p->maxAcceleration);

    if (s1 == 0 & s2 == 0)
    {
        runData_p->stepV0 = (moveSig_p->speedEnd - moveSig_p->speedStart) / runData_p->totalSteps;
        runData_p->stepsV0 = runData_p->totalSteps;
        runData_p->stepsVE = runData_p->totalSteps;
    }
    else 
    { 
        if (s0 > MIN_LENGTH)
        {
            runData_p->stepsV0 = (int)((double)runData_p->totalSteps * (s0 / length));
            if (runData_p->stepsV0 > 0)
            {
                runData_p->stepV0 = (vMaxReal - moveSig_p->speedStart) / runData_p->stepsV0;
            }                  
        }
        else
        {
            runData_p->stepsV0 = 0;
        }
        int stepsV2 = 0;
        if (s2 > MIN_LENGTH)
        {
            //segments++;
            stepsV2 = (int)((double)runData_p->totalSteps * (s2 / length));
            if (stepsV2 > 0)
            {
                runData_p->stepVE = (moveSig_p->speedEnd - vMaxReal) / stepsV2;
            }
        }

        runData_p->stepsVE = runData_p->totalSteps - stepsV2;
    }
    runData_p->vMax = vMaxReal;
  }
  else
  {
    runData_p->totalSteps = 1;
  }
  

}

void MoveProcessor_c::InitArc(MoveProcData_st* runData_p, MoveData_st*  moveData_p)
{

  runData_p->totalSteps = 100;

  iPoint3D rCentrePoint = moveData_p->centrePoint.ToPlane( moveData_p->planeMode  );
  runData_p->rEndPoint  = moveData_p->endPoint.ToPlane( moveData_p->planeMode  );
  runData_p->endApos = moveData_p->endApos;
  runData_p->planeMode = moveData_p->planeMode;
  runData_p->moveType = moveData_p->moveType;
  
  runData_p->progress = 0;
  

  Point3D sP(0,0,0);
  Point3D cP(0.001 * (rCentrePoint.x), 0.001 * (rCentrePoint.y), 0.001 * (rCentrePoint.z));
  Point3D eP(0.001 * (runData_p->rEndPoint.x), 0.001 * (runData_p->rEndPoint.y), 0.001 * (runData_p->rEndPoint.z));

  runData_p->rCentrePoint = cP;

  Vector3D vR1 = sP-cP;

  runData_p->radius = sqrtf(vR1.X * vR1.X + vR1.Y * vR1.Y);
 
  float startAngle = atan2f(vR1.Y,vR1.X);
  float endAngle;
  if(moveData_p->turns > 0 )
  {
    if (moveData_p->clockwise == true)
    {
      endAngle = startAngle - 2 * moveData_p->turns * M_PI;
    }
    else
    {
      endAngle = startAngle + 2 * moveData_p->turns * M_PI;
    }
  }
  else
  {
    
    Vector3D vR2 = eP-cP;
    endAngle = atan2f(vR2.Y,vR2.X);

    if (moveData_p->clockwise == true)
    {      
      if (startAngle < endAngle) { endAngle -= (M_PI * 2); }

    }
    else
    {
      
      if (startAngle > endAngle) { endAngle += (M_PI * 2); }
    }
  }
  runData_p->startAngle = startAngle;
  runData_p->endAngle = endAngle;
  runData_p->stepAngle = (endAngle - startAngle) / runData_p->totalSteps;
  runData_p->actAngle = startAngle + runData_p->stepAngle;

  float lengthA = (startAngle - endAngle) * runData_p->radius;
  runData_p->height = (eP.Z);

  float length = sqrtf(lengthA*lengthA + runData_p->height*runData_p->height);


  float s0,s1,s2,vMaxReal;
  GetRampDistances(&s0,&s1,&s2,&vMaxReal,length,moveData_p->startSpeed,moveData_p->endSpeed,moveData_p->maxSpeed,moveData_p->maxA);

  if (s1 == 0 & s2 == 0)
  {
      runData_p->stepV0 = (moveData_p->endSpeed - moveData_p->startSpeed) / runData_p->totalSteps;
      runData_p->stepsV0 = runData_p->totalSteps;
      runData_p->stepsVE = runData_p->totalSteps;
  }
  else 
  { 
      if (s0 > MIN_LENGTH)
      {
          runData_p->stepsV0 = (int)((double)runData_p->totalSteps * (s0 / length));
          if (runData_p->stepsV0 > 0)
          {
              runData_p->stepV0 = (vMaxReal - moveData_p->startSpeed) / runData_p->stepsV0;
          }                  
      }
      else
      {
          runData_p->stepsV0 = 0;
      }
      int stepsV2 = 0;
      if (s2 > MIN_LENGTH)
      {
          //segments++;
          stepsV2 = (int)((double)runData_p->totalSteps * (s2 / length));
          if (stepsV2 > 0)
          {
              runData_p->stepVE = (moveData_p->endSpeed - vMaxReal) / stepsV2;
          }
      }

      runData_p->stepsVE = runData_p->totalSteps - stepsV2;
  }
  runData_p->startSpeed = moveData_p->startSpeed;
  runData_p->endSpeed = moveData_p->endSpeed;
  runData_p->vMax = vMaxReal;


}
void MoveProcessor_c::InitArcNew(MoveProcData_st* runData_p, CNC_moveSig_c* moveSig_p)
{
  runData_p->endPoint.x = moveSig_p->x;
  runData_p->endPoint.y = moveSig_p->y;
  runData_p->endPoint.z = moveSig_p->z;
  runData_p->endApos = moveSig_p->a;

  #if TEST_AXE_PIPELINE == 1
  printf("ARC: POS=(%d,%d,%d,%d) CEN=(%d,%d,%d) ",  
  runData_p->endPoint.x,runData_p->endPoint.y,runData_p->endPoint.z,runData_p->endApos,
  moveSig_p->cx,moveSig_p->cy,moveSig_p->cz);
  printf("ACTPOS = (%d,%d,%d,%d)\n", position[0],position[1],position[2],position[3]);
  #endif

  runData_p->moveType = moveSig_p->moveType;  
  runData_p->progress = 0;
  runData_p->rActPoint = iPoint3D(0,0,0);
  runData_p->aMax = moveSig_p->maxAcceleration;

  float minSpeed = Axe_c::GetMinSpeed();
  if (moveSig_p->speedStart < minSpeed) {moveSig_p->speedStart = minSpeed;}
  if (moveSig_p->speedEnd < minSpeed) {moveSig_p->speedEnd = minSpeed;}
  if (moveSig_p->maxSpeed < minSpeed) {moveSig_p->maxSpeed = minSpeed;}


  GetRotMatrix(&runData_p->rotMatrix,&runData_p->rotMatrixInverted,&moveSig_p->rotVector);

  //RotMatrix_c::PrintMatrix(&runData_p->rotMatrix);
  
  Point3D sP(0.001 * (position[0]), 0.001 * (position[1]), 0.001 * (position[2]));
  Point3D cP(0.001 * (moveSig_p->cx), 0.001 * (moveSig_p->cy), 0.001 * (moveSig_p->cz));
  Point3D eP(0.001 * (moveSig_p->x), 0.001 * (moveSig_p->y), 0.001 * (moveSig_p->z));

/*
  Point3D cP_plane_tmp = runData_p->rotMatrixInverted1 * cP;
  Point3D eP_plane_tmp = runData_p->rotMatrixInverted1 * eP;
  Point3D cP_plane = runData_p->rotMatrixInverted2 * cP_plane_tmp;
  Point3D eP_plane = runData_p->rotMatrixInverted2 * eP_plane_tmp;*/
  Point3D cP_plane = runData_p->rotMatrixInverted * cP;
  Point3D eP_plane = runData_p->rotMatrixInverted * eP;

  Point3D sP_Plane = runData_p->rotMatrixInverted * sP;

  Vector3D vR1 = sP_Plane-cP_plane; 
  runData_p->rCentrePoint = cP_plane;
  runData_p->radius = sqrtf(vR1.X * vR1.X + vR1.Y * vR1.Y);

  //runData_p->totalSteps = 10 * runData_p->radius;
  
  float startAngle = atan2f(vR1.Y,vR1.X);
  float endAngle;
  if(moveSig_p->turns > 0 )
  {
    endAngle = startAngle - 2 * moveSig_p->turns * M_PI;
  }
  else
  {    
    Vector3D vR2 = eP_plane-cP_plane;
    endAngle = atan2f(vR2.Y,vR2.X);
   
    if (startAngle < endAngle) { endAngle -= (M_PI * 2); }
  }

  float prefferedLengthStep = 0.1;

  float prefferedAngleStep = prefferedLengthStep / runData_p->radius;
  runData_p->totalSteps = -(endAngle - startAngle)/prefferedAngleStep;
  if(runData_p->totalSteps < 3)
  {
    runData_p->totalSteps = 3;
  }


  runData_p->startAngle = startAngle;
  runData_p->endAngle = endAngle;
  runData_p->stepAngle = (endAngle - startAngle) / runData_p->totalSteps;
  runData_p->actAngle = startAngle + runData_p->stepAngle;

  float lengthA = (startAngle - endAngle) * runData_p->radius;
  runData_p->height = (eP_plane.Z - sP_Plane.Z);
  runData_p->startZ = sP_Plane.Z;

  float length = sqrtf(lengthA*lengthA + runData_p->height*runData_p->height);

  float s0,s1,s2,vMaxReal;
  GetRampDistances(&s0,&s1,&s2,&vMaxReal,length,moveSig_p->speedStart,moveSig_p->speedEnd,moveSig_p->maxSpeed,moveSig_p->maxAcceleration);

  if (s1 == 0 & s2 == 0)
  {
      runData_p->stepV0 = (moveSig_p->speedEnd - moveSig_p->speedStart) / runData_p->totalSteps;
      runData_p->stepsV0 = runData_p->totalSteps;
      runData_p->stepsVE = runData_p->totalSteps;
  }
  else 
  { 
      if (s0 > MIN_LENGTH)
      {
          runData_p->stepsV0 = (int)((double)runData_p->totalSteps * (s0 / length));
          if (runData_p->stepsV0 > 0)
          {
              runData_p->stepV0 = (vMaxReal - moveSig_p->speedStart) / runData_p->stepsV0;
          }                  
      }
      else
      {
          runData_p->stepsV0 = 0;
      }
      int stepsV2 = 0;
      if (s2 > MIN_LENGTH)
      {
          //segments++;
          stepsV2 = (int)((double)runData_p->totalSteps * (s2 / length));
          if (stepsV2 > 0)
          {
              runData_p->stepVE = (moveSig_p->speedEnd - vMaxReal) / stepsV2;
          }
      }

      runData_p->stepsVE = runData_p->totalSteps - stepsV2;
  }
  runData_p->startSpeed = moveSig_p->speedStart;
  runData_p->endSpeed = moveSig_p->speedEnd;
  runData_p->vMax = vMaxReal;



}

int MoveProcessor_c::Round(int val, int s)
{
  int res = (val/s) * s;
  return res;
}



SEGMENT_REULT_et MoveProcessor_c::GetLineSimpleMove(SimpleMove_st* sMove_p, MoveProcData_st* runData_p, int* scales,SurfaceOffset_c* surfaceOffset)
{



  bool lastSeg = true;

  if(runData_p->totalSteps > 1)
  {
    if(runData_p->progress < runData_p->totalSteps-1)
    {
      lastSeg = false;
    }

    int i = runData_p->progress;
    
    if(i < runData_p->stepsV0)
    {
        sMove_p->startSpeed = runData_p->startSpeed + i * runData_p->stepV0;
        sMove_p->endSpeed = sMove_p->startSpeed + runData_p->stepV0;
    }
    else if(i < runData_p->stepsVE)
    {
        sMove_p->startSpeed = runData_p->vMax;
        sMove_p->endSpeed = runData_p->vMax;
    }
    else
    {
        sMove_p->startSpeed = runData_p->vMax + (i - (runData_p->stepsVE))  * runData_p->stepVE;
        sMove_p->endSpeed = sMove_p->startSpeed +  runData_p->stepVE;
    }

    if(i== 0)
    {
      sMove_p->startSpeed = runData_p->startSpeed;
    }

    if(lastSeg == true)
    {
      sMove_p->x = runData_p->rEndPoint.x - position[0];
      sMove_p->y = runData_p->rEndPoint.y - position[1];
      sMove_p->z = runData_p->rEndPoint.z - position[2];
      sMove_p->a = runData_p->endApos     - position[3];
      sMove_p->endSpeed = runData_p->endSpeed;
    }
    else
    {
      sMove_p->x = runData_p->step[0];
      sMove_p->y = runData_p->step[1];
      sMove_p->z = runData_p->step[2];
      sMove_p->a = runData_p->step[3];
    }
    runData_p->progress++;
  }
  else
  {
    sMove_p->x = runData_p->rEndPoint.x - position[0];
    sMove_p->y = runData_p->rEndPoint.y - position[1];
    sMove_p->z = runData_p->rEndPoint.z - position[2];
    sMove_p->a = runData_p->endApos     - position[3];


    sMove_p->startSpeed = runData_p->startSpeed;
    sMove_p->endSpeed = runData_p->endSpeed;
  }


  /* round move data to possible value */


  int zOffset = surfaceOffset->AddSurfaceOffset(position[0]+sMove_p->x, position[1]+sMove_p->y);
  sMove_p->z += zOffset;

  sMove_p->x = Round(sMove_p->x,scales[0]);
  sMove_p->y = Round(sMove_p->y,scales[1]);
  sMove_p->z = Round(sMove_p->z,scales[2]);  
  sMove_p->a = Round(sMove_p->a,scales[3]);



  if((sMove_p->x == 0) && (sMove_p->y == 0)  && (sMove_p->z == 0)  && (sMove_p->a == 0))
  {
    return IGNORE_LAST_SEGMENT;
  }


  position[0] += sMove_p->x;
  position[1] += sMove_p->y;
  position[2] += sMove_p->z;
  position[3] += sMove_p->a;
  

 #if TEST_AXE_PIPELINE == 1
  printf("Seg L: (%d,%d,%d,%d) ",sMove_p->x,sMove_p->y,sMove_p->z,sMove_p->a);
  printf("ACTPOS = (%d,%d,%d,%d)\n", position[0],position[1],position[2],position[3]);
  #endif


  if(lastSeg)
  {
    return LAST_SEGMENT;
  }
  else
  {
    return MORE_SEGMENTS;
  }
}

SEGMENT_REULT_et MoveProcessor_c::GetArcSimpleMove(SimpleMove_st* sMove_p, MoveProcData_st* runData_p,  int* scales)
{
  SEGMENT_REULT_et lastSegment = MORE_SEGMENTS;

  if(runData_p->progress == runData_p->totalSteps-1)
  {
    lastSegment = LAST_SEGMENT;
  }


  int i = runData_p->progress;
  if(i < runData_p->stepsV0)
  {
      sMove_p->startSpeed = runData_p->startSpeed + i * runData_p->stepV0;
      sMove_p->endSpeed = sMove_p->startSpeed + runData_p->stepV0;
  }
  else if(i < runData_p->stepsVE)
  {
      sMove_p->startSpeed = runData_p->vMax;
      sMove_p->endSpeed = runData_p->vMax;
  }
  else
  {
      sMove_p->startSpeed = runData_p->vMax + (i - (runData_p->stepsVE))  * runData_p->stepVE;
      sMove_p->endSpeed = sMove_p->startSpeed +  runData_p->stepVE;
  }
  if(lastSegment)
  {
      sMove_p->endSpeed = runData_p->endSpeed;
  }



  iVector3D move;
  if(lastSegment)
  {

    move = runData_p->rEndPoint - runData_p->rActPoint;
  }
  else
  {

    float x = cosf(runData_p->actAngle) * runData_p->radius + runData_p->rCentrePoint.X;
    float y = sinf(runData_p->actAngle) * runData_p->radius + runData_p->rCentrePoint.Y;
    float z = runData_p->height * ( (float)i / (float)runData_p->totalSteps);

    iPoint3D newPoint( (int)(x*1000) ,(int)(y*1000),(int)(z*1000));

    newPoint.x = Round(newPoint.x,scales[0]);
    newPoint.y = Round(newPoint.y,scales[1]);
    newPoint.z = Round(newPoint.z,scales[2]);
    //newPoint.a = Round(newPoint.a,scales[3]);

    move = newPoint - runData_p->rActPoint;
    runData_p->rActPoint  = newPoint;
  }

  move = move.FromPlane(runData_p->planeMode);

  sMove_p->x = move.x;
  sMove_p->y = move.y;
  sMove_p->z = move.z;
  sMove_p->a = 0;

  

  runData_p->progress++;
  runData_p->actAngle += runData_p->stepAngle;

  return lastSegment;
}
/*
void PrintSimpleMove(iPoint3D* actPos,SimpleMove_st* move_p)
{

  printf("SMOVE:(%d,%d,%d) (%d,%d,%d,%d), v0=%.2f ve=%.2f\n", 
  actPos->x,
  actPos->y,
  actPos->z,
  move_p->x,
  move_p->y,
  move_p->z,
  move_p->a,
  move_p->startSpeed,
  move_p->endSpeed);

}*/


SEGMENT_REULT_et MoveProcessor_c::GetArcNewSimpleMove(SimpleMove_st* sMove_p, MoveProcData_st* runData_p,  int* scales,SurfaceOffset_c* surfaceOffset)
{
  SEGMENT_REULT_et result = MORE_SEGMENTS;

  if(runData_p->progress == runData_p->totalSteps-1)
  {
    result = LAST_SEGMENT;
  }


  int i = runData_p->progress;
  if(i < runData_p->stepsV0)
  {
      sMove_p->startSpeed = runData_p->startSpeed + i * runData_p->stepV0;
      sMove_p->endSpeed = sMove_p->startSpeed + runData_p->stepV0;
  }
  else if(i < runData_p->stepsVE)
  {
      sMove_p->startSpeed = runData_p->vMax;
      sMove_p->endSpeed = runData_p->vMax;
  }
  else
  {
      sMove_p->startSpeed = runData_p->vMax + (i - (runData_p->stepsVE))  * runData_p->stepVE;
      sMove_p->endSpeed = sMove_p->startSpeed +  runData_p->stepVE;
  }
  if(result == LAST_SEGMENT)
  {
      sMove_p->endSpeed = runData_p->endSpeed;
  }



  iVector3D move;
  if(result == LAST_SEGMENT)
  {

    move.x = runData_p->endPoint.x - position[0];
    move.y = runData_p->endPoint.y - position[1];
    move.z = runData_p->endPoint.z - position[2];
  }
  else
  {

    float x = cosf(runData_p->actAngle) * runData_p->radius + runData_p->rCentrePoint.X;
    float y = sinf(runData_p->actAngle) * runData_p->radius + runData_p->rCentrePoint.Y;
    float z = runData_p->height * ( (float)i / (float)runData_p->totalSteps) + runData_p->startZ;

    Point3D pt(x,y,z);

    pt = runData_p->rotMatrix * pt;

    iPoint3D newPoint( (int)(pt.X*1000) ,(int)(pt.Y*1000),(int)(pt.Z*1000));


    move.x = newPoint.x - position[0];
    move.y = newPoint.y - position[1];
    move.z = newPoint.z - position[2];
  }

  int zOffset = surfaceOffset->AddSurfaceOffset(position[0]+sMove_p->x, position[1]+sMove_p->y);
  sMove_p->z += zOffset;

  move.x = Round(move.x,scales[0]);
  move.y = Round(move.y,scales[1]);
  move.z = Round(move.z,scales[2]);
  //move.a = Round(move.a,scales[3]);




  position[0] += move.x;
  position[1] += move.y;
  position[2] += move.z;
  //position[3] += move.a;


  sMove_p->x = move.x;
  sMove_p->y = move.y;
  sMove_p->z = move.z;
  sMove_p->a = 0;

  //printf("Seg A: (%d,%d,%d,%d)\n",sMove_p->x,sMove_p->y,sMove_p->z,sMove_p->a);

  runData_p->progress++;
  runData_p->actAngle += runData_p->stepAngle;

  if((sMove_p->x == 0) && (sMove_p->y == 0)  && (sMove_p->z == 0)  && (sMove_p->a == 0))
  {
    if(result == LAST_SEGMENT)
    {
      return IGNORE_LAST_SEGMENT;
    }
    else
    {
      return IGNORE_SEGMENT;
    }
  }

  //printf("Move %d/%d: ",runData_p->progress , runData_p->totalSteps);
  //PrintSimpleMove(&runData_p->rActPoint,sMove_p);
  //vTaskDelay(1);

  return result;
}



void MoveProcessor_c::GetRotMatrix(RotMatrix_c* rM, RotMatrix_c* rMInv, Vector3D* vector_p)
{
  RotMatrix_c rotMatrix;

  if((vector_p->X == 0) && (vector_p->Y == 0))
  {

    if(vector_p->Z < 0)
    {
      /* simple inversion */
      rotMatrix(1,1) = -1;
      rotMatrix(2,2) = -1;
    }
    else
    {
      /*nothing to do */

    }
    *rM = rotMatrix;
    *rMInv = rotMatrix;
  }
  else
  {
    float angleZ = atan2f(vector_p->Y,vector_p->X);
    float angleY = acosf(vector_p->Z/vector_p->Length());

    RotMatrix_c rotY(angleY,'Y');
    RotMatrix_c rotZ(angleZ,'Z');

    RotMatrix_c rotYinv(-angleY,'Y');
    RotMatrix_c rotZinv(-angleZ,'Z');
    *rM =   rotZ * rotY;
    *rMInv =   rotYinv * rotZinv;

  }

}




void MoveProcessor_c::PrintSimpleMove(iPoint3D* actPos,SimpleMove_st* move_p)
{

  printf("SMOVE:(%d,%d,%d) (%d,%d,%d,%d), v0=%.2f ve=%.2f\n", 
  actPos->x,
  actPos->y,
  actPos->z,
  move_p->x,
  move_p->y,
  move_p->z,
  move_p->a,
  move_p->startSpeed,
  move_p->endSpeed);

}





