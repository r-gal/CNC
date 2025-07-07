#include <stdio.h>
#include <stdlib.h>
#include <cstring>



#define TEST_AXE 
#include "CNC2_MoveProcessor.hpp"









int CNC2_MoveProcessorTest(void)
{

  printf("test2\n");





        CNC_moveSig_c* sig_p = new CNC_moveSig_c;
        sig_p->seqNo = 0;

        sig_p->speedStart = 0 ;
        sig_p->speedEnd = 0 ;
        sig_p->maxSpeed = 20 ;
        sig_p->maxAcceleration = 20 ;

        sig_p->turns = 2;
      
        sig_p->x = 0;
        sig_p->y = 0;
        sig_p->z = 0;
        sig_p->a = 0;

        sig_p->cx = 0;
        sig_p->cy = 10000;
        sig_p->cz = 0;

        sig_p->rotVector.X = 0;
        sig_p->rotVector.Y = 0;
        sig_p->rotVector.Z = 1000;

        sig_p->moveType = MOVE_ARC2;



  MoveProcData_st runData;

  MoveProcessor_c::InitArcNew(&runData, sig_p);


  iPoint3D actPos(0,0,0);

  bool lastSegment = false;

  while(lastSegment == false)
  {
    

    SimpleMove_st simpleMove;

    lastSegment = MoveProcessor_c::GetArcNewSimpleMove(&simpleMove,&runData,10,10,10);

    actPos.x += simpleMove.x;
    actPos.y += simpleMove.y;
    actPos.z += simpleMove.z;

    MoveProcessor_c::PrintSimpleMove(&actPos, &simpleMove);

  }

/*

  Vector3D rotVector(1,0,0);

  Point3D endPoint(1,2,3);

  RotMatrix_c rM;
  RotMatrix_c rMInv;

  MoveProcessor_c::GetRotMatrix(&rM,&rMInv,&rotVector);

  printf("rot Matrix:\n");
  RotMatrix_c::PrintMatrix(&rM);

  Point3D rotPoint = rM * endPoint;

  printf("rot Matrix inv:\n");
  RotMatrix_c::PrintMatrix(&rMInv);

  printf("Point: (%.2f,%.2f,%.2f)\n",rotPoint.X,rotPoint.Y,rotPoint.Z);

  Point3D rot2Point = rMInv *rotPoint;

  printf("Point: (%.2f,%.2f,%.2f)\n",rot2Point.X,rot2Point.Y,rot2Point.Z);

  Point3D rot3PointTmp = rMInv * endPoint;
  Point3D rot3Point = rM * rot3PointTmp;

  printf("Point: (%.2f,%.2f,%.2f)\n",rot3Point.X,rot3Point.Y,rot3Point.Z);
*/
  return 1;


}