#pragma once
#include <cstdint>
struct TwoWire {
  void begin(int=0,int=0,uint32_t=0){}
  void setClock(uint32_t){}
  void beginTransmission(uint8_t){}
  size_t write(uint8_t){return 1;}
  uint8_t endTransmission(bool=true){return 0;}
  uint8_t requestFrom(uint8_t,uint8_t){return 0;}
  int available(){return 0;}
  int read(){return 0;}
};
extern TwoWire Wire;
// Вторая шина (IMU, 2026-09-26). На стенде никто не отвечает — IMU «не найден».
static TwoWire Wire1;
