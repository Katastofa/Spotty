#pragma once
#include <cstdint>
class Adafruit_PWMServoDriver {
public:
  Adafruit_PWMServoDriver(uint8_t=0x40){}
  bool begin(){return true;}
  void reset(){}
  void setPWMFreq(float){}
  void setOscillatorFrequency(uint32_t f){_osc=f;}
  uint32_t getOscillatorFrequency(){return _osc;}
  uint32_t _osc = 25000000;   // умолчание библиотеки
  void setPWM(uint8_t,uint16_t,uint16_t){}
  void writeMicroseconds(uint8_t,uint16_t){}
  uint8_t readPrescale(){return 121;}
};
