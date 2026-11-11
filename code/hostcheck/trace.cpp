// ===========================================================================
// trace — выгрузка траектории суставов из НАСТОЯЩЕЙ прошивки.
// ---------------------------------------------------------------------------
// Собирается вместе с gait_test.ino (см. build.sh), поэтому считает ровно тот
// код, который заливается в ESP32-S3. Копии походки здесь нет: цикл ниже —
// это тот же тик, что в MODE_WALK, вызывающий те же gaitPose / setTarget /
// slewToTargets. Единственное, чего нет, — подачи ШИМ.
//
//   ./build.sh trace -- gait=0 span=30 lift=15 vel=60 swing=900 out=traj.csv
//
// На stdout уходит JSON со сводкой (период, duty, пик скорости сустава,
// геометрия звеньев, пределы каналов в градусах, флаги отказа), в файл out —
// CSV: время и 12 углов в градусах, в том виде, в каком они ушли бы в серво
// (после клампа по жёсткому пределу и после ограничителя скорости).
//
//   geom=1   — только сводка, без прогона и без CSV.
//
// ПЕРЕНОС КОРПУСА ПО ФАЗЕ:
//   shiftgain=1                включить ТАБЛИЦУ ПРОШИВКИ (то же, что 'B 1'
//                              на роботе). Амплитуду считает прошивка из
//                              своей же геометрии; с 2026-09-12 продольная
//                              база ИЗМЕРЕНА (186 мм), подставлять нечего.
//                              Сводка печатает shift_fw_y — таблицу, снятую
//                              с самой прошивки.
//   baselong=<мм>              ПОДМЕНИТЬ базу: свип по чувствительности или
//                              проверка «а что было бы». Сводка помечает это
//                              в base_long_source, и молча такой прогон за
//                              измеренный выдать нельзя.
//   sx=/sy=                    СТАРАЯ ветка: подмена сдвига стендом. Нужна
//                              только для форм, которых в прошивке нет.
//                              Вместе с shiftgain= запрещена.
// ===========================================================================
#include "Arduino.h"
#include "Wire.h"
#include "spotty_config.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

bool HOST_QUIET = true;
HostSerial Serial;
TwoWire Wire;

// --- то, что предоставляет прошивка -----------------------------------------
enum { HIP = 0, THIGH = 1, KNEE = 2, NJ = 3 };

// Раскладка пресета походки. Это ОБЪЯВЛЕНИЕ чужой таблицы, не её копия:
// поля читаются из gait_test.ino, значения там же и живут.
struct GaitDef {
  const char *name;
  const char *note;
  float phase[4];
  float duty;
  float ampThigh, ampKnee;
  float cycleMs;
};
extern const GaitDef GAITS[];
extern const uint16_t TICK_MS;

extern int   pEngine;
extern float pHeightMm, pSpanMm, pLiftMm, pPenetMm, pVelMmS, pSwingMs;
extern float pCrabDeg, pShiftXMm, pShiftYMm, pRollDeg, pPitchDeg, pYawDeg;
extern float pDuty, pAmpThigh, pAmpKnee, pCycleMs, pAmpHip;
extern float linkRatio, pCrouchDeg, pTrimDeg, pMaxRateDps;
extern bool  pFlatPath, pSmoothSwing;
extern int   pTurn;
extern bool  ikFail;
extern unsigned char gaitIdx;
extern uint32_t limitHits;
extern float curAngle[4][3], tgtAngle[4][3];

extern void  gaitPose(unsigned char, float, float *);
extern void  loadGaitPreset();
extern void  setTargetsToPose();
extern void  recalcCartesianTiming();
extern void  slewToTargets(float);
extern void  setTarget(unsigned char, unsigned char, float);
extern float peakRateDps(float);
extern const float L_FEMUR_MM, L_TIBIA_MM, L_HIP_MM, R_FOOT_MM;
extern const float BASE_LAT_MM;
extern float BASE_LONG_MM;          // НЕ const в копии стенда, см. build.sh
extern const bool BASE_LONG_IS_MEASURED;   // флаг прошивки, не догадка стенда

// Перенос корпуса по фазе. Это функции ПРОШИВКИ: стенд их зовёт, а не
// повторяет. Таблица в сводке ниже — то, что посчитает робот, а не то, что
// стенд считает правильным.
extern float pShiftGain;
extern float bodyShiftAmpMm();
extern float bodyShiftYAt(float);

// Трюки (команда 'e', 2026-09-28). Раскладка таблицы — ОБЪЯВЛЕНИЕ, как у
// GaitDef; кадры и логика живут в прошивке, стенд зовёт те же функции.
enum Mode { MODE_IDLE, MODE_ARM, MODE_RAMP, MODE_WALK, MODE_HOLD };
extern Mode mode;
extern bool havePose, engaged;
struct TrickKey { uint16_t ms; int16_t d[4][3]; };
struct TrickDef { const char *name, *ru, *note; const TrickKey *k; uint8_t n; bool endSit; bool hip; };
extern uint8_t trickCount();
extern const TrickDef *trickDef(uint8_t);
extern bool  trickStart(uint8_t, unsigned long);
extern void  rampModeTick(unsigned long);
extern int8_t trickIdx;
extern uint8_t trickKey;
extern char  trickErr[];
extern float TRICK_HIP_OUT;
extern bool  trickHipOk;

// Профили ходьбы (команда 'M', 2026-09-30) — объявление таблицы прошивки.
struct GaitProfile {
  const char *name, *ru, *note;
  uint8_t gait, engine, air;
  float   t, k, d, T, q, b, m;
  uint8_t f, S;
  float   H, P, C, V, W, B;
};
extern uint8_t profileCount();
extern const GaitProfile *profileDef(uint8_t);

// --- разбор аргументов вида ключ=значение ------------------------------------
static float argf(int argc, char **argv, const char *key, float def) {
  const size_t n = strlen(key);
  for (int i = 1; i < argc; i++)
    if (!strncmp(argv[i], key, n) && argv[i][n] == '=')
      return (float)atof(argv[i] + n + 1);
  return def;
}

// Таблица через запятую: sx=0,12,0,-12 — четыре узла, равномерно по фазе.
#define MAXTAB 16
static int argtab(int argc, char **argv, const char *key, float *out) {
  const size_t n = strlen(key);
  for (int i = 1; i < argc; i++) {
    if (strncmp(argv[i], key, n) || argv[i][n] != '=') continue;
    const char *s = argv[i] + n + 1;
    int k = 0;
    while (*s && k < MAXTAB) {
      out[k++] = (float)atof(s);
      const char *c = strchr(s, ',');
      if (!c) break;
      s = c + 1;
    }
    return k;
  }
  return 0;
}

/* Строка в JSON. Нужна ровно из-за одного: пути на Windows выглядят как
 * C:\Users\... , а обратная косая в JSON — начало escape-последовательности.
 * Сырой путь превращал сводку в синтаксически неверный JSON, и разборщик
 * падал на «Invalid \escape» — причём только на Windows, где стенд и нужен
 * для живых ползунков. Экранируем то, что обязана экранировать спецификация:
 * обратную косую, кавычку и управляющие символы. */
static void jputs(const char *s) {
  for (; s && *s; s++) {
    if (*s == '\\' || *s == '"') { putchar('\\'); putchar(*s); }
    else if ((unsigned char)*s < 0x20) printf("\\u%04x", (unsigned)(unsigned char)*s);
    else putchar(*s);
  }
}

// Циклическая линейная интерполяция таблицы по фазе 0..1.
static float tabAt(const float *tab, int n, float phase) {
  if (n <= 0) return 0.0f;
  if (n == 1) return tab[0];
  float u = phase - floorf(phase);
  float f = u * (float)n;
  int i0 = (int)f;
  if (i0 >= n) i0 = n - 1;
  const float frac = f - (float)i0;
  const int i1 = (i0 + 1) % n;
  return tab[i0] + (tab[i1] - tab[i0]) * frac;
}

int main(int argc, char **argv) {
  const int   gait     = (int)argf(argc, argv, "gait", 0);
  const float cycles   = argf(argc, argv, "cycles", 4.0f);
  const float settleMs = argf(argc, argv, "settle", 400.0f);
  const bool  geomOnly = argf(argc, argv, "geom", 0) > 0.5f;

  gaitIdx = (unsigned char)gait;
  loadGaitPreset();            // фазы, duty и амплитуды пресета

  pEngine      = (int)argf(argc, argv, "engine",  1);      // 1 = декартов
  pHeightMm    = argf(argc, argv, "height",  190.0f);
  pSpanMm      = argf(argc, argv, "span",     30.0f);
  pLiftMm      = argf(argc, argv, "lift",     15.0f);
  pPenetMm     = argf(argc, argv, "penet",     0.0f);
  pVelMmS      = argf(argc, argv, "vel",      60.0f);
  pSwingMs     = argf(argc, argv, "swing",   900.0f);
  pCrabDeg     = argf(argc, argv, "crab",      0.0f);
  pShiftXMm    = argf(argc, argv, "shiftx",    0.0f);
  pShiftYMm    = argf(argc, argv, "shifty",    0.0f);
  pRollDeg     = argf(argc, argv, "roll",      0.0f);
  pTrimDeg     = argf(argc, argv, "trim",     17.0f);
  pMaxRateDps  = argf(argc, argv, "maxrate", 100.0f);
  pTurn        = (int)argf(argc, argv, "turn", 0);
  pFlatPath    = argf(argc, argv, "flat",   0) > 0.5f;
  pSmoothSwing = argf(argc, argv, "smooth", 0) > 0.5f;
  linkRatio    = L_TIBIA_MM / L_FEMUR_MM;

  /* ПРОДОЛЬНАЯ БАЗА И ПЕРЕНОС КОРПУСА.
   *
   * baselong= подаётся В ПЕРЕМЕННУЮ ПРОШИВКИ, а не в отдельную копию: так
   * амплитуду переноса считает bodyShiftAmpMm() прошивки, из той же
   * геометрии, по той же формуле, и разойтись им негде. На роботе эта
   * переменная равна нулю до замера M4, и прошивка на нуле отказывает —
   * поэтому прогон с baselong= обязан помечаться как опирающийся на
   * ПРИНЯТОЕ, а не на измеренное число. Поле base_long_source ниже.
   *
   * shiftgain= — тот же параметр, что команда 'B' на роботе. Стенд НЕ
   * подменяет таблицу: он включает ту, что живёт в прошивке. */
  const float baseLongArg = argf(argc, argv, "baselong", -1.0f);
  const bool  baseLongSubst = (baseLongArg >= 0.0f);
  if (baseLongSubst) BASE_LONG_MM = baseLongArg;
  pShiftGain   = argf(argc, argv, "shiftgain", 0.0f);

  recalcCartesianTiming();     // duty и период — ВЫВОДИМЫЕ из vel/swing/span

  // Угловой движок: подмена пресета походки теми же переменными, что
  // команды t k d T q a (2026-09-30). Отрицательное значение — не трогать.
  { float v;
    if ((v = argf(argc, argv, "amp",    -1.0f)) >= 0.0f) pAmpThigh  = v;
    if ((v = argf(argc, argv, "knee",   -1.0f)) >= 0.0f) pAmpKnee   = v;
    if ((v = argf(argc, argv, "duty",   -1.0f)) >= 0.0f) pDuty      = v;
    if ((v = argf(argc, argv, "period", -1.0f)) >= 0.0f) pCycleMs   = v;
    if ((v = argf(argc, argv, "crouch", -1.0f)) >= 0.0f) pCrouchDeg = v;
    if ((v = argf(argc, argv, "hipamp", -1.0f)) >= 0.0f) pAmpHip    = v; }

  // Пределы каналов в градусах — прямо из таблицы прошивки, не переписаны.
  float lim[4][3][2];
  for (int leg = 0; leg < 4; leg++)
    for (int j = 0; j < 3; j++)
      spottyAngleLimits(SPOTTY_CH[leg][j], lim[leg][j][0], lim[leg][j][1]);

  printf("{\n");
  // base_long_measured — флаг САМОЙ ПРОШИВКИ (SPOTTY_BASE_LONG_MEASURED), а не
  // догадка стенда. Симулятор ставит статус числа по нему, а не по величине:
  // ненулевая база может быть и подставленной (baselong=), и измеренной, и
  // путать эти два случая — ровно то, ради чего заведена политика статусов.
  printf("  \"geom_mm\": {\"femur\": %.4f, \"tibia\": %.4f, \"hip\": %.4f,\n"
         "               \"foot_r\": %.4f, \"base_lat\": %.4f, \"base_long\": %.4f,\n"
         "               \"base_long_measured\": %s},\n",
         L_FEMUR_MM, L_TIBIA_MM, L_HIP_MM, R_FOOT_MM, BASE_LAT_MM, BASE_LONG_MM,
         BASE_LONG_IS_MEASURED ? "true" : "false");
  printf("  \"legs\": [\"FR\", \"FL\", \"BR\", \"BL\"],\n");
  printf("  \"limits_deg\": [");
  for (int leg = 0; leg < 4; leg++)
    for (int j = 0; j < 3; j++)
      printf("%s[%.3f, %.3f]", (leg || j) ? ", " : "", lim[leg][j][0], lim[leg][j][1]);
  printf("],\n");
  printf("  \"gait\": \"%s\", \"phase\": [%.3f, %.3f, %.3f, %.3f],\n",
         GAITS[gaitIdx].name, GAITS[gaitIdx].phase[0], GAITS[gaitIdx].phase[1],
         GAITS[gaitIdx].phase[2], GAITS[gaitIdx].phase[3]);
  printf("  \"duty\": %.5f, \"cycle_ms\": %.3f, \"tick_ms\": %u,\n",
         pDuty, pCycleMs, (unsigned)TICK_MS);
  printf("  \"peak_rate_dps\": %.2f, \"max_rate_dps\": %.2f,\n",
         peakRateDps(pCycleMs), pMaxRateDps);

  // Перенос корпуса по фазе — ТАБЛИЦА ПРОШИВКИ, снятая с неё же вызовом.
  // Копии здесь нет: bodyShiftYAt живёт в gait_test.ino.
  printf("  \"base_long_source\": \"%s\",\n",
         baseLongSubst ? "ПОДСТАВЛЕНА стендом (свип или замена замера)"
                       : (BASE_LONG_IS_MEASURED ? "прошивка, ИЗМЕРЕНО (M4)"
                                                : "прошивка, НЕ ИЗМЕРЕНА (M4 не снят)"));
  printf("  \"shift_gain\": %.4f, \"shift_amp_mm\": %.4f,\n",
         pShiftGain, bodyShiftAmpMm());
  printf("  \"shift_fw_y\": [");
  for (int i = 0; i < 8; i++)
    printf("%s%.4f", i ? ", " : "", bodyShiftYAt((float)i / 8.0f));
  printf("],\n");

  // Список трюков — из таблицы прошивки. Пульт строит по нему кнопки.
  printf("  \"tricks\": [");
  for (uint8_t i = 0; i < trickCount(); i++) {
    const TrickDef *T = trickDef(i);
    unsigned ms = 0;
    for (uint8_t k = 0; k < T->n; k++) ms += T->k[k].ms;
    printf("%s{\"name\": \"", i ? ", " : "");
    jputs(T->name); printf("\", \"ru\": \""); jputs(T->ru); printf("\", \"note\": \""); jputs(T->note);
    printf("\", \"keys\": %u, \"ms\": %u, \"end_sit\": %s, \"hip\": %s}",
           (unsigned)T->n, ms, T->endSit ? "true" : "false", T->hip ? "true" : "false");
  }
  printf("],\n");

  // Профили ходьбы — числа прямо из spotty_profiles.h. Пульт в СИМУЛЯЦИИ
  // ставит их теми же ключами (amp knee duty period crouch …), что и 'M'.
  printf("  \"profiles\": [");
  for (uint8_t i = 0; i < profileCount(); i++) {
    const GaitProfile *P = profileDef(i);
    printf("%s{\"name\": \"", i ? ", " : ""); jputs(P->name);
    printf("\", \"ru\": \""); jputs(P->ru); printf("\", \"note\": \""); jputs(P->note);
    printf("\", \"gait\": %u, \"engine\": %u, \"air\": %s, \"t\": %g, \"k\": %g, \"d\": %g, \"T\": %g, "
           "\"q\": %g, \"b\": %g, \"m\": %g, \"f\": %u, \"S\": %u, \"H\": %g, \"P\": %g, \"C\": %g, "
           "\"V\": %g, \"W\": %g, \"B\": %g}",
           P->gait, P->engine, P->air ? "true" : "false", P->t, P->k, P->d, P->T, P->q, P->b, P->m,
           P->f, P->S, P->H, P->P, P->C, P->V, P->W, P->B);
  }
  printf("],\n");

  if (geomOnly) { printf("  \"traced\": false\n}\n"); return 0; }

  /* --- ПОДМЕНА СДВИГА СТЕНДОМ (sx=/sy=) --------------------------------------
   * Историческая ветка, и теперь она ВТОРАЯ по старшинству. Пока таблицы в
   * прошивке не было, стенд ставил pShiftXMm / pShiftYMm сам перед каждым
   * вызовом gaitPose — так и выяснилось, какой таблица должна быть. Теперь
   * таблица живёт в прошивке и включается shiftgain=, а sx=/sy= остались для
   * ОДНОГО: проверять формы, которых в прошивке НЕТ, прежде чем их заводить.
   *
   * Вместе они запрещены. Два источника сдвига в одном прогоне — это ровно
   * тот случай, когда потом не скажешь, чьё поведение измерено. */
  float sxTab[MAXTAB], syTab[MAXTAB];
  const int nSx = argtab(argc, argv, "sx", sxTab);
  const int nSy = argtab(argc, argv, "sy", syTab);
  if ((nSx || nSy) && pShiftGain > 0.0f) {
    printf("  \"error\": \"sx=/sy= (подмена стендом) и shiftgain= (таблица прошивки) "
           "вместе запрещены: было бы два источника сдвига\"\n}\n");
    return 3;
  }
  if (nSx) pShiftXMm = tabAt(sxTab, nSx, 0.0f);
  if (nSy) pShiftYMm = tabAt(syTab, nSy, 0.0f);

  // --- стартовая поза ---------------------------------------------------------
  ikFail = false; limitHits = 0;
  setTargetsToPose();
  bool poseFail = ikFail;
  for (int leg = 0; leg < 4; leg++)
    for (int j = 0; j < 3; j++) curAngle[leg][j] = tgtAngle[leg][j];

  const char *out = "traj.csv";
  for (int i = 1; i < argc; i++)
    if (!strncmp(argv[i], "out=", 4)) out = argv[i] + 4;
  FILE *f = fopen(out, "w");
  if (!f) {
    printf("  \"error\": \"не открыть ");
    jputs(out);
    printf("\"\n}\n");
    return 2;
  }

  fprintf(f, "t_s");
  const char *LN[4] = { "FR", "FL", "BR", "BL" };
  const char *JN[3] = { "hip", "thigh", "knee" };
  for (int leg = 0; leg < 4; leg++)
    for (int j = 0; j < 3; j++) fprintf(f, ",%s_%s", LN[leg], JN[j]);
  fprintf(f, ",phase\n");

  const float dt = (float)TICK_MS / 1000.0f;
  int nSettle = (int)(settleMs / (float)TICK_MS + 0.5f);
  int nWalk   = (int)(cycles * pCycleMs / (float)TICK_MS + 0.5f);
  float t = 0.0f;

  // Стояние в позе: физике нужно осесть до того, как поедет походка.
  for (int k = 0; k < nSettle; k++) {
    slewToTargets(pMaxRateDps);
    fprintf(f, "%.4f", t);
    for (int leg = 0; leg < 4; leg++)
      for (int j = 0; j < 3; j++) fprintf(f, ",%.5f", curAngle[leg][j]);
    fprintf(f, ",-1\n");
    t += dt;
  }

  /* ТРЮК: trick=<номер>. Тот же тик, что MODE_RAMP прошивки (rampModeTick):
   * пролог, кадры, эпилог. hipok=0 — как на роботе до 'e hip 1' (откажет). */
  const int trickArg = (int)argf(argc, argv, "trick", -1.0f);
  if (trickArg >= 0) {
    engaged = true; havePose = true; mode = MODE_HOLD;
    trickHipOk = argf(argc, argv, "hipok", 1.0f) > 0.5f;
    TRICK_HIP_OUT = argf(argc, argv, "hipsign", 1.0f) < 0.0f ? -1.0f : 1.0f;
    unsigned long now = 100000UL;
    const bool ok = trickStart((uint8_t)trickArg, now);
    int rows = nSettle, keyRow[64], nKey = 0, lastKey = -1;
    const int maxRows = (int)(90000.0f / (float)TICK_MS);
    if (ok) {
      int tail = -1;
      for (int k = 0; k < maxRows; k++) {
        now += TICK_MS;
        if (mode == MODE_RAMP) rampModeTick(now);
        if (trickIdx >= 0 && (int)trickKey != lastKey && nKey < 64) { keyRow[nKey++] = rows; lastKey = trickKey; }
        fprintf(f, "%.4f", t);
        for (int leg = 0; leg < 4; leg++)
          for (int j = 0; j < 3; j++) fprintf(f, ",%.5f", curAngle[leg][j]);
        fprintf(f, ",-1\n");
        t += dt; rows++;
        if (mode != MODE_RAMP && tail < 0) tail = (int)(300.0f / (float)TICK_MS);
        if (tail >= 0 && tail-- == 0) break;
      }
    }
    fclose(f);
    printf("  \"traced\": %s, \"csv\": \"", ok ? "true" : "false");
    jputs(out);
    printf("\", \"rows\": %d, \"settle_rows\": %d,\n", rows, nSettle);
    printf("  \"trick\": %d, \"trick_ok\": %s, \"trick_err\": \"", trickArg, ok ? "true" : "false");
    jputs(trickErr);
    printf("\", \"end_sit\": %s, \"key_rows\": [",
           (trickArg < (int)trickCount() && trickDef((uint8_t)trickArg)->endSit) ? "true" : "false");
    for (int i = 0; i < nKey; i++) printf("%s%d", i ? ", " : "", keyRow[i]);
    printf("],\n  \"pose_ik_fail\": %s, \"ik_fail\": false, \"limit_hits\": %u\n}\n",
           poseFail ? "true" : "false", (unsigned)limitHits);
    return 0;                         // отказ — в trick_ok/trick_err, не кодом
  }

  // Тик походки — тот же, что в MODE_WALK прошивки.
  ikFail = false;
  float gaitDone = 0.0f;
  int ikFailTick = -1;
  for (int k = 0; k < nWalk; k++) {
    gaitDone += (float)TICK_MS / pCycleMs;
    const float phase = gaitDone - floorf(gaitDone);
    if (nSx) pShiftXMm = tabAt(sxTab, nSx, phase);
    if (nSy) pShiftYMm = tabAt(syTab, nSy, phase);
    float ang[NJ];
    for (unsigned char leg = 0; leg < 4; leg++) {
      gaitPose(leg, GAITS[gaitIdx].phase[leg] + phase, ang);
      for (unsigned char j = 0; j < 3; j++) setTarget(leg, j, ang[j]);
    }
    if (ikFail && ikFailTick < 0) ikFailTick = k;
    slewToTargets(pMaxRateDps);
    fprintf(f, "%.4f", t);
    for (int leg = 0; leg < 4; leg++)
      for (int j = 0; j < 3; j++) fprintf(f, ",%.5f", curAngle[leg][j]);
    fprintf(f, ",%.5f\n", phase);
    t += dt;
  }
  fclose(f);

  printf("  \"traced\": true, \"csv\": \"");
  jputs(out);
  printf("\", \"rows\": %d, \"settle_rows\": %d,\n", nSettle + nWalk, nSettle);
  printf("  \"pose_ik_fail\": %s, \"ik_fail\": %s, \"ik_fail_tick\": %d,\n",
         poseFail ? "true" : "false", ikFail ? "true" : "false", ikFailTick);
  printf("  \"shift_x\": [");
  for (int i = 0; i < nSx; i++) printf("%s%.3f", i ? ", " : "", sxTab[i]);
  printf("], \"shift_y\": [");
  for (int i = 0; i < nSy; i++) printf("%s%.3f", i ? ", " : "", syTab[i]);
  printf("],\n");
  printf("  \"limit_hits\": %u\n}\n", (unsigned)limitHits);
  return 0;
}
