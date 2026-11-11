#pragma once
/*
 * spotty_imu.h — IMU как ИЗМЕРИТЕЛЬНЫЙ прибор. Проект Spotty, 2026-09-26.
 * ---------------------------------------------------------------------------
 * ЧТО ДЕЛАЕТ
 *   - находит чип на отдельной шине Wire1 (пины в spotty_config.h);
 *   - крен и тангаж корпуса комплементарным фильтром, курс — интегралом гиро;
 *   - за прогон 'w' — сводка: средний тангаж (это и есть дифферент для трима
 *     'b'), размах качки, увод курса за цикл;
 *   - аварийный стоп по наклону, ВЫКЛЮЧЕН по умолчанию ('imu stop <град>').
 *
 * ЧЕГО НЕ ДЕЛАЕТ
 *   Не влияет на траекторию. Обратная связь в походку — следующий шаг, после
 *   того как телеметрия покажет, что показания живут под вибрацией серво.
 *
 * ЧИПЫ
 *   MPU-6050/6500/9250/9255 и совместимые клоны (ICM-20600/20602) — полностью:
 *     одна карта регистров акселерометра и гиро.
 *   BNO055 — полностью: берётся его вектор гравитации (уже без линейных
 *     ускорений) и гиро, дальше тот же фильтр.
 *   ICM-20948 — определяется, но пока не читается (банки регистров).
 *
 * УГЛЫ (оси корпуса, см. SPOTTY_IMU_AX_* в конфиге)
 *   крен   +  правый борт вниз
 *   тангаж +  нос вверх        (как у команды 'E')
 *   курс   +  нос влево        (как у команды 'J'); относительный, дрейфует
 * ---------------------------------------------------------------------------
 */
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "spotty_config.h"

static_assert(SPOTTY_IMU_SDA != SPOTTY_I2C_SDA && SPOTTY_IMU_SDA != SPOTTY_I2C_SCL &&
              SPOTTY_IMU_SCL != SPOTTY_I2C_SDA && SPOTTY_IMU_SCL != SPOTTY_I2C_SCL,
              "IMU должен быть на своей шине, не на шине PCA9685");

enum SpottyImuChip : uint8_t { IMU_NONE, IMU_MPU, IMU_BNO055, IMU_ICM20948 };

static const uint16_t SPOTTY_IMU_PERIOD_MS = 6;     // ~166 Гц, раз в два тика
static const float    SPOTTY_IMU_ALPHA     = 0.98f; // доля гиро в фильтре

static TwoWire &spImuBus = Wire1;
static SpottyImuChip spImuChip = IMU_NONE;
static uint8_t  spImuAddr = 0;
static uint8_t  spImuWho  = 0;
static bool     spImuOk   = false;
static uint32_t spImuErr  = 0;           // ошибок чтения подряд
static uint32_t spImuErrTotal = 0;

// сырые в осях корпуса: ускорение в g, угловая скорость в °/с
static float spImuA[3] = {0, 0, 1};      // вперёд, влево, вверх
static float spImuG[3] = {0, 0, 0};      // крен-, тангаж-, курс-скорость
static float spImuBias[3] = {0, 0, 0};   // смещение гиро в осях МОДУЛЯ
static bool  spImuBiasOk = false;

static float spImuRoll = 0, spImuPitch = 0, spImuYaw = 0;
static float spImuRoll0 = 0, spImuPitch0 = 0;  // перекос крепления ('imu zero')
static uint32_t spImuLastUs = 0, spImuLastMs = 0;
static bool  spImuFirst = true;

static float spImuStopDeg = 0.0f;        // 0 = стоп по наклону выключен
static bool  spImuMon = false;
static uint32_t spImuMonMs = 0;

// статистика прогона
static bool  spImuRun = false;
static uint32_t spImuRunN = 0;
static double spImuRunPitchSum = 0, spImuRunRollSum = 0;
static float spImuRunRollMin = 0, spImuRunRollMax = 0;
static float spImuRunPitchMin = 0, spImuRunPitchMax = 0;
static float spImuRunYaw0 = 0, spImuRunGyroMax = 0;
static float spImuRunCycles = 0;

// ------------------------------- шина --------------------------------------
static bool spImuWrite(uint8_t reg, uint8_t v) {
  spImuBus.beginTransmission(spImuAddr);
  spImuBus.write(reg); spImuBus.write(v);
  return spImuBus.endTransmission() == 0;
}
static bool spImuReadAt(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  spImuBus.beginTransmission(addr);
  spImuBus.write(reg);
  if (spImuBus.endTransmission(false) != 0) return false;
  if (spImuBus.requestFrom((uint8_t)addr, (uint8_t)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)spImuBus.read();
  return true;
}
static bool spImuRead(uint8_t reg, uint8_t *buf, uint8_t n) { return spImuReadAt(spImuAddr, reg, buf, n); }

// Ось модуля (1..3, со знаком) -> значение из тройки модуля.
static inline float spImuPick(const float m[3], int8_t code) {
  int8_t i = code < 0 ? -code : code;
  float v = m[i - 1];
  return code < 0 ? -v : v;
}

// ------------------------------- чипы --------------------------------------
static const char *spImuName() {
  switch (spImuChip) {
    case IMU_MPU:      return spImuWho == 0x68 ? "MPU-6050" : spImuWho == 0x70 ? "MPU-6500"
                            : spImuWho == 0x71 ? "MPU-9250" : spImuWho == 0x73 ? "MPU-9255"
                            : "MPU-совместимый";
    case IMU_BNO055:   return "BNO055";
    case IMU_ICM20948: return "ICM-20948";
    default:           return "нет";
  }
}

static bool spImuMpuWhoOk(uint8_t w) {
  return w == 0x68 || w == 0x70 || w == 0x71 || w == 0x73 || w == 0x72 ||
         w == 0x98 || w == 0x12 || w == 0x11;
}

static bool spImuDetect() {
  uint8_t v = 0;
  for (uint8_t a = 0x68; a <= 0x69; a++) {
    if (spImuReadAt(a, 0x75, &v, 1) && spImuMpuWhoOk(v)) { spImuAddr = a; spImuWho = v; spImuChip = IMU_MPU; return true; }
    if (spImuReadAt(a, 0x00, &v, 1) && v == 0xEA)         { spImuAddr = a; spImuWho = v; spImuChip = IMU_ICM20948; return true; }
  }
  for (uint8_t a = 0x28; a <= 0x29; a++)
    if (spImuReadAt(a, 0x00, &v, 1) && v == 0xA0)         { spImuAddr = a; spImuWho = v; spImuChip = IMU_BNO055; return true; }
  // Что-то отвечает, но не опознано — назвать, чтобы не гадать.
  for (uint8_t a = 0x68; a <= 0x69; a++)
    if (spImuReadAt(a, 0x75, &v, 1)) { spImuAddr = a; spImuWho = v; return false; }
  return false;
}

static bool spImuInitChip() {
  if (spImuChip == IMU_MPU) {
    bool ok = spImuWrite(0x6B, 0x80);            // сброс
    delay(100);
    ok = spImuWrite(0x6B, 0x01);                 // проснуться, такт от гиро X
    delay(10);
    ok = ok && spImuWrite(0x1A, 0x03);           // DLPF ~44 Гц
    ok = ok && spImuWrite(0x19, 0x04);           // 1 кГц / 5 = 200 Гц
    ok = ok && spImuWrite(0x1B, 0x08);           // гиро ±500 °/с
    ok = ok && spImuWrite(0x1C, 0x08);           // аксель ±4 g
    return ok;
  }
  if (spImuChip == IMU_BNO055) {
    bool ok = spImuWrite(0x3D, 0x00);            // режим CONFIG
    delay(25);
    ok = ok && spImuWrite(0x3E, 0x00);           // питание normal
    ok = ok && spImuWrite(0x07, 0x00);           // страница 0
    ok = ok && spImuWrite(0x3B, 0x00);           // единицы: м/с², °/с, градусы
    ok = ok && spImuWrite(0x3D, 0x08);           // IMUPLUS: аксель+гиро, без магнитометра
    delay(25);
    return ok;
  }
  return false;
}

// Прочитать тройки МОДУЛЯ: ускорение (g), гиро (°/с), без смещения.
static bool spImuReadModule(float acc[3], float gyr[3]) {
  uint8_t b[14];
  if (spImuChip == IMU_MPU) {
    if (!spImuRead(0x3B, b, 14)) return false;
    for (int i = 0; i < 3; i++) {
      acc[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]) / 8192.0f;       // ±4 g
      gyr[i] = (int16_t)((b[8 + 2 * i] << 8) | b[8 + 2 * i + 1]) / 65.5f; // ±500 °/с
    }
    return true;
  }
  if (spImuChip == IMU_BNO055) {
    if (!spImuRead(0x14, b, 6)) return false;                              // гиро, 1/16 °/с
    for (int i = 0; i < 3; i++) gyr[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8)) / 16.0f;
    if (!spImuRead(0x2E, b, 6)) return false;                              // гравитация, 1/100 м/с²
    for (int i = 0; i < 3; i++) acc[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8)) / 100.0f / 9.80665f;
    return true;
  }
  return false;
}

// ---------------------------- калибровка -----------------------------------
// Смещение гиро: среднее за ~1 с. Робот обязан стоять неподвижно.
template <class Out>
static bool spottyImuCalGyro(Out &out) {
  if (!spImuOk) { out.println("IMU нет — калибровать нечего."); return false; }
  double s[3] = {0, 0, 0}, s2[3] = {0, 0, 0};
  int n = 0;
  float a[3], g[3];
  for (int i = 0; i < 200; i++) {
    if (spImuReadModule(a, g)) { for (int k = 0; k < 3; k++) { s[k] += g[k]; s2[k] += (double)g[k] * g[k]; } n++; }
    delay(5);
  }
  if (n < 100) { out.printf("калибровка гиро: прочитано %d из 200 — шина сбоит.\n", n); return false; }
  float sd = 0;
  for (int k = 0; k < 3; k++) {
    spImuBias[k] = (float)(s[k] / n);
    float var = (float)(s2[k] / n - (s[k] / n) * (s[k] / n));
    if (var > 0 && sqrtf(var) > sd) sd = sqrtf(var);
  }
  spImuBiasOk = true;
  out.printf("гиро: смещение %.2f %.2f %.2f °/с, шум %.2f °/с по %d замерам\n",
             spImuBias[0], spImuBias[1], spImuBias[2], sd, n);
  if (sd > 1.5f) out.println("[!] шум велик — робот двигался? Повторить 'imu cal' в покое.");
  spImuFirst = true; spImuYaw = 0;
  return true;
}

static void spottyImuNvsLoad() {
  Preferences p;
  p.begin("spotty", true);
  spImuRoll0  = p.getFloat("imur0", 0.0f);
  spImuPitch0 = p.getFloat("imup0", 0.0f);
  p.end();
}

// ------------------------------- старт -------------------------------------
template <class Out>
static void spottyImuBegin(Out &out) {
  out.println();
  out.printf("--- IMU (Wire1: SDA GPIO%u, SCL GPIO%u, INT GPIO%u — не используется) ---\n",
             SPOTTY_IMU_SDA, SPOTTY_IMU_SCL, SPOTTY_IMU_INT);
  spImuBus.begin(SPOTTY_IMU_SDA, SPOTTY_IMU_SCL, SPOTTY_IMU_I2C_HZ);
  spottyImuNvsLoad();
  spImuOk = false;
  if (!spImuDetect()) {
    if (spImuAddr) out.printf("на 0x%02X отвечает неизвестный чип (WHO_AM_I 0x%02X). IMU выключен.\n", spImuAddr, spImuWho);
    else           out.println("IMU не найден на 0x68/0x69/0x28/0x29. Проверить провода. Робот работает без него.");
    return;
  }
  out.printf("найден %s на 0x%02X (id 0x%02X)\n", spImuName(), spImuAddr, spImuWho);
  if (spImuChip == IMU_ICM20948) { out.println("ICM-20948 пока не поддержан (банки регистров). IMU выключен."); return; }
  if (!spImuInitChip()) { out.println("инициализация не прошла. IMU выключен."); return; }
  spImuOk = true;
  out.printf("оси: вперёд=%+d влево=%+d вверх=%+d (1=X 2=Y 3=Z) — НЕ ПРОВЕРЕНЫ, см. 'imu mon'\n",
             SPOTTY_IMU_AX_FWD, SPOTTY_IMU_AX_LEFT, SPOTTY_IMU_AX_UP);
  spottyImuCalGyro(out);                          // серво ещё мягкие, робот в покое
  if (spImuRoll0 != 0 || spImuPitch0 != 0)
    out.printf("перекос крепления из флеша: крен %+.2f°, тангаж %+.2f°\n", spImuRoll0, spImuPitch0);
  else
    out.println("перекос крепления не записан ('imu zero' на ровном корпусе).");
}

// ------------------------------- тик ---------------------------------------
/*
 * Вызывать каждый тик. Сама читает не чаще SPOTTY_IMU_PERIOD_MS.
 * walking — идёт ли сейчас 'w'; cycles — сколько циклов пройдено (gaitDone).
 * Смена walking false->true начинает сводку, true->false её печатает.
 */
template <class Out>
static void spottyImuTick(bool walking, float cycles, Out &out) {
  if (!spImuOk) return;
  uint32_t ms = millis();
  if (ms - spImuLastMs < SPOTTY_IMU_PERIOD_MS) return;
  spImuLastMs = ms;

  float a[3], g[3];
  if (!spImuReadModule(a, g)) {
    spImuErrTotal++;
    if (++spImuErr == 50) out.println("[!] IMU: 50 ошибок чтения подряд. Показания заморожены.");
    return;
  }
  spImuErr = 0;
  if (spImuBiasOk) for (int k = 0; k < 3; k++) g[k] -= spImuBias[k];

  spImuA[0] = spImuPick(a, SPOTTY_IMU_AX_FWD);
  spImuA[1] = spImuPick(a, SPOTTY_IMU_AX_LEFT);
  spImuA[2] = spImuPick(a, SPOTTY_IMU_AX_UP);
  const float gF = spImuPick(g, SPOTTY_IMU_AX_FWD);
  const float gL = spImuPick(g, SPOTTY_IMU_AX_LEFT);
  const float gU = spImuPick(g, SPOTTY_IMU_AX_UP);
  spImuG[0] = gF;          // крен: вращение вокруг «вперёд», + правый борт вниз
  spImuG[1] = -gL;         // тангаж: вокруг «влево» по правилу правой руки нос идёт ВНИЗ
  spImuG[2] = gU;          // курс: вокруг «вверх», + нос влево

  const float accRoll  = degrees(atan2f(spImuA[1], spImuA[2]));
  const float accPitch = degrees(atan2f(spImuA[0], sqrtf(spImuA[1] * spImuA[1] + spImuA[2] * spImuA[2])));

  uint32_t us = micros();
  float dt = spImuFirst ? 0.0f : (us - spImuLastUs) * 1e-6f;
  spImuLastUs = us;
  if (dt > 0.1f) dt = 0.1f;
  if (spImuFirst) { spImuRoll = accRoll; spImuPitch = accPitch; spImuFirst = false; }
  else {
    spImuRoll  = SPOTTY_IMU_ALPHA * (spImuRoll  + spImuG[0] * dt) + (1 - SPOTTY_IMU_ALPHA) * accRoll;
    spImuPitch = SPOTTY_IMU_ALPHA * (spImuPitch + spImuG[1] * dt) + (1 - SPOTTY_IMU_ALPHA) * accPitch;
    spImuYaw  += spImuG[2] * dt;
  }
  const float roll = spImuRoll - spImuRoll0, pitch = spImuPitch - spImuPitch0;

  // ---- сводка прогона ----
  if (walking && !spImuRun) {
    spImuRun = true; spImuRunN = 0; spImuRunPitchSum = spImuRunRollSum = 0;
    spImuRunRollMin = spImuRunRollMax = roll; spImuRunPitchMin = spImuRunPitchMax = pitch;
    spImuRunYaw0 = spImuYaw; spImuRunGyroMax = 0; spImuRunCycles = 0;
  }
  if (walking) {
    spImuRunN++; spImuRunPitchSum += pitch; spImuRunRollSum += roll;
    if (roll  < spImuRunRollMin)  spImuRunRollMin  = roll;
    if (roll  > spImuRunRollMax)  spImuRunRollMax  = roll;
    if (pitch < spImuRunPitchMin) spImuRunPitchMin = pitch;
    if (pitch > spImuRunPitchMax) spImuRunPitchMax = pitch;
    float gm = fmaxf(fabsf(spImuG[0]), fmaxf(fabsf(spImuG[1]), fabsf(spImuG[2])));
    if (gm > spImuRunGyroMax) spImuRunGyroMax = gm;
    spImuRunCycles = cycles;
  }
  if (!walking && spImuRun) {
    spImuRun = false;
    if (spImuRunN > 10) {
      const float dYaw = spImuYaw - spImuRunYaw0;
      out.println("--- IMU за прогон ---");
      out.printf("  тангаж: среднее %+.1f°  (%+.1f…%+.1f, размах %.1f°)  %s\n",
                 spImuRunPitchSum / spImuRunN, spImuRunPitchMin, spImuRunPitchMax,
                 spImuRunPitchMax - spImuRunPitchMin,
                 spImuRunPitchSum / spImuRunN < -2 ? "<- нос ВНИЗ" : spImuRunPitchSum / spImuRunN > 2 ? "<- нос ВВЕРХ" : "");
      out.printf("  крен:   среднее %+.1f°  (%+.1f…%+.1f, размах %.1f°)\n",
                 spImuRunRollSum / spImuRunN, spImuRunRollMin, spImuRunRollMax,
                 spImuRunRollMax - spImuRunRollMin);
      if (spImuRunCycles > 0.5f)
        out.printf("  курс:   %+.1f° за %.1f цикл(а) = %+.2f°/цикл %s\n", dYaw, spImuRunCycles,
                   dYaw / spImuRunCycles, dYaw > 2 ? "(уводит ВЛЕВО)" : dYaw < -2 ? "(уводит ВПРАВО)" : "");
      out.printf("  пик угловой скорости %.0f °/с, ошибок чтения за всё время %lu\n",
                 spImuRunGyroMax, (unsigned long)spImuErrTotal);
    }
  }

  if (spImuMon && ms - spImuMonMs >= 200) {
    spImuMonMs = ms;
    out.printf("IMU крен %+6.1f  тангаж %+6.1f  курс %+7.1f   a[вп,вл,вв] %+.2f %+.2f %+.2f g\n",
               roll, pitch, spImuYaw, spImuA[0], spImuA[1], spImuA[2]);
  }
}

// Аварийный стоп по наклону: true, если порог задан и превышен.
static bool spottyImuTiltFault(float *rollOut, float *pitchOut) {
  if (!spImuOk || spImuStopDeg <= 0) return false;
  const float r = spImuRoll - spImuRoll0, p = spImuPitch - spImuPitch0;
  if (rollOut) *rollOut = r;
  if (pitchOut) *pitchOut = p;
  return fabsf(r) > spImuStopDeg || fabsf(p) > spImuStopDeg;
}

// ------------------------------ вывод --------------------------------------
template <class Out>
static void spottyImuStatus(Out &out) {
  if (!spImuOk) { out.printf("IMU: %s\n", spImuChip == IMU_NONE ? "не найден" : "найден, но выключен"); return; }
  out.printf("IMU %s: крен %+.1f°  тангаж %+.1f°  курс %+.1f° (отн.)  стоп по наклону: %s",
             spImuName(), spImuRoll - spImuRoll0, spImuPitch - spImuPitch0, spImuYaw,
             spImuStopDeg > 0 ? "" : "выкл");
  if (spImuStopDeg > 0) out.printf("%.0f°", spImuStopDeg);
  out.println();
}

template <class Out>
static void spottyImuHelp(Out &out) {
  out.println("  --- IMU (только измерение) ---");
  out.println("  imu          показать углы и состояние");
  out.println("  imu mon      вкл/выкл строку углов 5 раз в секунду (проверка осей)");
  out.println("  imu cal      калибровка гиро — робот НЕПОДВИЖЕН");
  out.println("  imu zero     корпус РОВНО на столе -> записать перекос крепления");
  out.println("  imu zero clr стереть перекос крепления");
  out.println("  imu y0       обнулить курс");
  out.println("  imu stop <°> аварийный стоп по наклону на ходу, 0 = выкл (по умолчанию)");
}

template <class Out>
static void spottyImuCommand(const char *a, Out &out) {
  while (*a == ' ') a++;
  if (!*a) { spottyImuStatus(out); return; }
  if (!spImuOk && strncmp(a, "stop", 4)) { out.println("IMU не работает — см. вывод при старте."); return; }

  if (!strncmp(a, "mon", 3)) { spImuMon = !spImuMon; out.printf("монитор IMU %s\n", spImuMon ? "ВКЛ" : "выкл"); return; }
  if (!strncmp(a, "cal", 3)) { spottyImuCalGyro(out); return; }
  if (!strncmp(a, "y0", 2))  { spImuYaw = 0; out.println("курс обнулён"); return; }
  if (!strncmp(a, "zero", 4)) {
    const char *r = a + 4; while (*r == ' ') r++;
    Preferences p; p.begin("spotty", false);
    if (!strncmp(r, "clr", 3)) {
      spImuRoll0 = spImuPitch0 = 0; p.remove("imur0"); p.remove("imup0");
      out.println("перекос крепления стёрт");
    } else {
      spImuRoll0 = spImuRoll; spImuPitch0 = spImuPitch;
      p.putFloat("imur0", spImuRoll0); p.putFloat("imup0", spImuPitch0);
      out.printf("перекос крепления записан: крен %+.2f°, тангаж %+.2f°\n", spImuRoll0, spImuPitch0);
      out.println("Корпус должен был лежать РОВНО (не стоять на ногах) — иначе записан наклон робота.");
    }
    p.end();
    return;
  }
  if (!strncmp(a, "stop", 4)) {
    float v = atof(a + 4);
    if (v < 0 || v > 80) { out.println("imu stop <0..80>, 0 = выкл"); return; }
    spImuStopDeg = v;
    if (v > 0) out.printf("стоп по наклону: крен или тангаж больше %.0f° на ходу -> удержание позы\n", v);
    else       out.println("стоп по наклону выключен");
    return;
  }
  spottyImuHelp(out);
}
