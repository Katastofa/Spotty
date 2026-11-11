#pragma once
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <cstdint>
// Строковый макрос Arduino: на ESP32 кладёт литерал во флеш, на хосте не
// делает ничего. Нужен, чтобы скетчи собирались стендом без правок.
#ifndef F
#define F(x) (x)
#endif
#define PI 3.1415926535897932384626433832795f
#define HALF_PI 1.5707963267948966f
static inline float degrees(float r){return r*57.295779513f;}
static inline float radians(float d){return d*0.017453292519943295f;}
template<class T,class A,class B> static inline T constrain(T x,A lo,B hi){return x<(T)lo?(T)lo:(x>(T)hi?(T)hi:x);}
static inline unsigned long millis(){return 0;}
static inline unsigned long micros(){return 0;}
static inline void delay(unsigned long){}
static inline void delayMicroseconds(unsigned long){}
static inline void yield(){}
// Прерывания и GPIO — для зонда генератора (spotty_osc.h, 2026-09-26).
// На стенде фронтов нет: замер честно отвечает «провод не подключён», K = 1.
#define IRAM_ATTR
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(m) ((void)(m))
#define portEXIT_CRITICAL(m) ((void)(m))
#define portENTER_CRITICAL_ISR(m) ((void)(m))
#define portEXIT_CRITICAL_ISR(m) ((void)(m))
#define INPUT_PULLDOWN 0x09
#define RISING 0x01
static inline void pinMode(int,int){}
static inline int  digitalPinToInterrupt(int p){return p;}
static inline void attachInterrupt(int, void(*)(), int){}
static inline void detachInterrupt(int){}
extern bool HOST_QUIET;
struct HostSerial {
  void begin(long){}
  int  available(){return 0;}
  int  read(){return -1;}
  void printf(const char*f,...){ if(HOST_QUIET) return; va_list a; va_start(a,f); vprintf(f,a); va_end(a);}
  void println(){ if(!HOST_QUIET) printf("\n");}
  void println(const char*s){ if(!HOST_QUIET) ::printf("%s\n",s);}
  void print(const char*s){ if(!HOST_QUIET) ::printf("%s",s);}
  size_t write(uint8_t c){ if(!HOST_QUIET) ::putchar((int)c); return 1; }
  size_t write(const uint8_t*b,size_t n){ for(size_t i=0;i<n;i++) write(b[i]); return n; }  // websyntax.sh (2026-09-28)
  void flush(){}
  operator bool(){return true;}
};
extern HostSerial Serial;
