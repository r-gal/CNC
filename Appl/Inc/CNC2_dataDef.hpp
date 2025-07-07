#ifndef CNC2_DAT_ADEF_H
#define CNC2_DAT_ADEF_H

#include "Vector.hpp"
#include "Matrix.hpp"
#include "CNC2_def.hpp"



struct MoveData_st
{
  MOVE_TYPE_et moveType;
  /* position data */
  iPoint3D centrePoint;
  iPoint3D endPoint;
  int endApos;
  PLANE_SELECT_et planeMode;
  bool clockwise;
  int turns;

  union{
  int delay;
  int spindleSpeed;
  };

  int maxSegLen;
  
  /* speed data */
  float maxA;
  float startSpeed;
  float endSpeed;
  float maxSpeed;


};






struct MoveProcData_st
{
  MOVE_TYPE_et moveType;

  /*common data*/

  float startSpeed;
  float endSpeed;
  float vMax;
  float aMax;

  /*dwel data */

  union{
  int delay;
  int spindleSpeed;
  };

  /*line data */

  int endApos;

  int step[NO_OF_AXES];

  int maxSegLen;

  /* arc data */


  int progress;
  float actAngle;
  iPoint3D rActPoint;


  float startAngle;
  float endAngle;
  float stepAngle;

  float radius;

  int totalSteps;
  int stepsV0;
  int stepsVE;
  float stepV0;
  float stepVE;

  Point3D rCentrePoint;
  float height;
  float startZ;

  iPoint3D rEndPoint;

  PLANE_SELECT_et planeMode;

  

  /* arc new data */

  Point3D centrePoint;
  iPoint3D endPoint;

  RotMatrix_c rotMatrix;
  RotMatrix_c rotMatrixInverted;
};

struct SimpleMove_st
{
  int x;
  int y;
  int z;
  int a;

  float startSpeed;
  float endSpeed;
};







#endif