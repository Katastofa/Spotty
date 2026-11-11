#include "Arduino.h"
#include "Wire.h"
#include <cstdio>
bool HOST_QUIET=true; HostSerial Serial; TwoWire Wire;
extern int pEngine; extern float pHeightMm,pSpanMm,pLiftMm,pPenetMm,pVelMmS,pSwingMs;
extern float pCrabDeg,pShiftXMm,pShiftYMm,pRollDeg,pDuty,pCycleMs;
extern void loadGaitPreset(); extern void recalcCartesianTiming();
extern float peakRateDps(float); extern unsigned char gaitIdx;
int main(){
  gaitIdx=1; loadGaitPreset();
  pEngine=1; pHeightMm=190; pPenetMm=0; pCrabDeg=0; pShiftXMm=0; pShiftYMm=0; pRollDeg=0;
  HOST_QUIET=false;
  printf("шаг±  подъём  скор  перенос | период duty | пик °/с\n");
  float spans[]={20,30,40}, lifts[]={10,15,20}, vels[]={40,60,80}, sws[]={500,700,900};
  for(float sp:spans) for(float li:lifts) for(float v:vels) for(float sw:sws){
    pSpanMm=sp; pLiftMm=li; pVelMmS=v; pSwingMs=sw; recalcCartesianTiming();
    float r=peakRateDps(pCycleMs);
    if(r<100.0f)
      printf("%4.0f %6.0f %5.0f %7.0f | %6.0f %.2f | %6.0f\n",sp,li,v,sw,pCycleMs,pDuty,r);
  }
  return 0;
}
