#ifndef CNC2_MOVE_PROCESSOR_H
#define CNC2_MOVE_PROCESSOR_H

#include "CNC2_dataDef.hpp"

#include "Vector.hpp"
#include "Matrix.hpp"

#include "CNC2_SurfaceOffset.hpp"

#define MIN_LENGTH (0.021)


class MoveProcessor_c
{

  static int Round(int val, int s);
  static void GetRampDistances(float* s0,float* s1, float* s2, float* vMaxReal,     float length, float v0, float vE, float Vmax, float aMax);

  static int position[NO_OF_AXES];


public:

  static void SetPosition(int idx,int value) { position[idx] = value; }
  static int GetPosition(int idx) { return position[idx]; }

  static SEGMENT_REULT_et GetArcSimpleMove(SimpleMove_st* sMove_p, MoveProcData_st* runData_p, int* scales);
  static SEGMENT_REULT_et GetArcNewSimpleMove(SimpleMove_st* sMove_p, MoveProcData_st* runData_p,  int* scales, SurfaceOffset_c* surfaceOffset);
  static SEGMENT_REULT_et GetLineSimpleMove(SimpleMove_st* sMove_p, MoveProcData_st* runData_p, int* scales, SurfaceOffset_c* surfaceOffset);

  

  static void InitArc(MoveProcData_st* runData_p, MoveData_st*  moveData_p);
  static void InitLine(MoveProcData_st* runData_p, CNC_moveSig_c* moveSig_p, int maxSegLength);
  static void InitDwel(MoveProcData_st* runData_p, MoveData_st*  moveData_p);
  static void InitArcNew(MoveProcData_st* runData_p, CNC_moveSig_c* moveSig_p);

  static void GetRotMatrix(RotMatrix_c* rM, RotMatrix_c* rmInv, Vector3D* vector_p);


  static void PrintSimpleMove(iPoint3D* actPos,SimpleMove_st* move_p);




};





#endif
