// ===========================================================================
// ikdump — сетка точек рабочей области и углы, которые даёт НАСТОЯЩАЯ legIK.
// Нужна ровно для одного: доказать, что URDF описывает ту же ногу, что и
// прошивка. Симулятор ставит эти углы в модель, берёт положение стопы и
// сравнивает с той точкой, из которой углы получены.
//
//   ./build.sh ikdump                 CSV на stdout: x,y,z,hip,thigh,knee
// ===========================================================================
#include "Arduino.h"
#include "Wire.h"
#include <cstdio>

bool HOST_QUIET = true;
HostSerial Serial;
TwoWire Wire;

extern bool legIK(float, float, float, float &, float &, float &);
extern const float L_HIP_MM;

int main() {
  printf("x,y,z,hip,thigh,knee\n");
  for (float x = -60; x <= 60.01f; x += 10)
    for (float y = L_HIP_MM - 25; y <= L_HIP_MM + 25.01f; y += 5)
      for (float z = 140; z <= 210.01f; z += 10) {
        float h, t, k;
        if (!legIK(x, y, z, h, t, k)) continue;
        printf("%.4f,%.4f,%.4f,%.6f,%.6f,%.6f\n", x, y, z, h, t, k);
      }
  return 0;
}
