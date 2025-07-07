 #include <stdio.h>
#include <stdlib.h>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "math.h"


#include "CNC2_SurfaceOffset.hpp"


void SurfaceOffset_c::Clear(void)
{
  xStart = 0;
  yStart = 0;
  xSize = 0;
  ySize = 0;
  xStep = 0;
  yStep = 0;
  active = false;
  if(array != nullptr)
  {
    delete[] array;
    array = nullptr;
  }
}

SurfaceOffset_c::SurfaceOffset_c()
{
  array = nullptr;
  Clear();    
}

void SurfaceOffset_c::Init(int xSize, int ySize, int xStep, int yStep, int xStart, int yStart)
{
  this->xSize = xSize;
  this->ySize = ySize;
  this->xStep = xStep;
  this->yStep = yStep;
  this->xStart = xStart;
  this->yStart = yStart;
  this->active = false;

  if((xSize > 0) && (ySize > 0))
  {
    if(xSize * ySize <= 1024)
    {
      array = new int[xSize * ySize];
      memset(array, 0, sizeof(int) * xSize *  ySize);
    }
  }
}

void SurfaceOffset_c::SetProbe(int x, int y, int val)
{
  if((x< xSize) && (y < ySize) && (array != nullptr))
  {
    array[x*xSize + y] = val;
  }
}

int SurfaceOffset_c::GetPoint(int x, int y)
{
  return array[x*xSize + y];

}

void SurfaceOffset_c::Activate(void) 
{
  if(array != nullptr)
  {
    active = true;
  }
}

void  SurfaceOffset_c::Deactivate(void)
{
  active = false;
}

int SurfaceOffset_c::AddSurfaceOffset(int x ,int y)
{
  if(active)
  {
    if((x > xStart) && (y > yStart))
    {
      int xTmp = x - xStart;
      int yTmp = y - yStart;

      int xIdx = xTmp / xStep;
      int yIdx = yTmp / yStep;

      if((xIdx < xSize) && (yIdx < ySize))
      {
        int xMod = xTmp % xStep;
        int yMod = yTmp % yStep;
        int xMod2 = xStep -xMod;
        int yMod2 = yStep -yMod;

        int p0 = GetPoint(xIdx,yIdx);
        int p1 = GetPoint(xIdx+1,yIdx);
        int p2 = GetPoint(xIdx,yIdx+1);
        int p3 = GetPoint(xIdx+1,yIdx+1);

        int av1 = p0 * xMod2 + p2 * xMod;
        int av2 = p1 * xMod2 + p3 * xMod;
        av1 /= xStep;
        av2 /= xStep;

        int av = av1 * yMod2 + av2 * yMod;

        int val = av /  yStep;

        return val;
      }
    }
  }
  return 0;
}

int SurfaceOffset_c::GetMaxSegLength(void)
{
  if(active)
  {
    int maxStep;
    if(yStep > xStep)
    {
      maxStep = xStep;
    }
    else
    {
      maxStep = yStep;
    }
    return maxStep/2;
  }
  else
  {
    return 0;
  }
}