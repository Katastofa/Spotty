#include "Arduino.h"
#include "Wire.h"
#include <cstdio>
bool HOST_QUIET=false; HostSerial Serial; TwoWire Wire;
extern int pEngine; extern float pHeightMm,pSpanMm,pLiftMm,pPenetMm,pVelMmS,pSwingMs;
extern float pCrabDeg,pShiftXMm,pShiftYMm,pRollDeg,pMaxRateDps;
extern bool preflight(bool); extern void loadGaitPreset(); extern void recalcCartesianTiming();
extern unsigned char gaitIdx; extern bool spottyLimitsSelfCheck(bool);
int main(){
  spottyLimitsSelfCheck(false);
  gaitIdx=1; loadGaitPreset();
  pEngine=1; pHeightMm=190; pSpanMm=30; pLiftMm=15; pPenetMm=0;
  pVelMmS=60; pSwingMs=900; pCrabDeg=0; pShiftXMm=0; pShiftYMm=0; pRollDeg=0;
  recalcCartesianTiming();
  printf("\n########## ЧТО НАПЕЧАТАЕТ 'p' ПРИ G 1, УМОЛЧАНИЯХ ##########\n");
  preflight(true);
  printf("\n########## ТО ЖЕ, НО ШАГ ±45 мм И СКОРОСТЬ 90 мм/с ##########\n");
  pSpanMm=45; pVelMmS=90; recalcCartesianTiming(); preflight(true);
  return 0;
}
