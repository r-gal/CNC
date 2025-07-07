#ifndef CNC2_SURFACE_OFFSET_H
#define CNC2_SURFACE_OFFSET_H

#include "SignalList.hpp"
#include "CommonDef.hpp"
#include "CNC2_dataDef.hpp"



class SurfaceOffset_c
{

  int xSize;
  int ySize;
  int xStep;
  int yStep;
  int xStart;
  int yStart;
  bool active;

  int* array;

  int GetPoint(int x, int y);

  public:

  void Clear(void);
  SurfaceOffset_c();
  void Init(int xSize, int ySize, int xStep, int yStep, int xStart, int yStart);
  void SetProbe(int x, int y, int val);
  void Activate(void) ;
  void Deactivate(void);

  int AddSurfaceOffset(int x, int y);

  int GetMaxSegLength(void);

};

#endif