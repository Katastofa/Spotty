#pragma once
/*
 * spotty_osc.h — поправка на уход генератора PCA9685.
 * Проект Spotty. Добавлено 2026-09-26. Общий для gait_test, servo_hold,
 * servo_calib (в папках скетчей — шимы на этот файл, как у spotty_config.h).
 * ---------------------------------------------------------------------------
 * ЧТО ЛЕЧИТ
 *   Длительность импульса PCA9685 = такты × (делитель+1) / f_ген. Генератор
 *   встроенный, RC, и плавает от температуры и питания. Уход на 1 % сдвигает
 *   импульс 1500 мкс на 15 мкс = 2.7° на КАЖДОМ суставе разом. Через
 *   JOINT_SIGN это видно как сдвиг колен и бёдер по БОРТУ, хипов — по
 *   ДИАГОНАЛИ. Нули в spotty_config.h при этом ни при чём.
 *
 * КАК
 *   Один свободный канал (SPOTTY_OSC_PROBE_CH) выдаёт меандр, ESP32 считает
 *   фронты на SPOTTY_OSC_PROBE_GPIO по своему кварцу:
 *       f_ген = f_ШИМ × 4096 × (делитель + 1)
 *   Эталон f_ref — частота генератора в тот момент, когда нули были ВЕРНЫ.
 *   Дальше каждый импульс умножается на
 *       K = f_ген_сейчас / f_ref
 *   и физическая длительность совпадает с днём калибровки.
 *
 *   Поправка ставится ПОСЛЕ всех клампов, на самом последнем шаге мкс -> такты.
 *   Пределы каналов при этом остаются физически верными: они сняты в тех же
 *   «мкс дня калибровки», что и нули.
 *
 *   ФОН (с вечера 2026-09-26): меандр на канале зонда идёт постоянно, и
 *   spottyOscTrackTick() раз в 2 с пересчитывает K по свежему замеру —
 *   шагом не больше 0.02 %, так что суставы не дёргаются. Ведётся только
 *   K «по зонду»; ручную, которую подбирает оператор, фон не трогает.
 *
 *   При K = 1.0 ровно число тактов побитово совпадает с прежним (умножение на
 *   1.0 в double точное) — без провода и без эталона всё как было.
 *
 * ЭТАЛОН, ЕСЛИ ИСТИННЫЙ НЕИЗВЕСТЕН
 *   Частота в день калибровки нулей не замерялась. Поэтому эталон ставится
 *   так: 'osc +' / 'osc -' двигают K, пока ВСЕ ноги разом не встанут ровно,
 *   затем 'osc save' запоминает f_ref = f_сейчас / K. Если одна ручка
 *   выправила все двенадцать суставов — диагноз «генератор» подтверждён.
 *   Если нет — дело в механике, и это тоже ответ.
 *
 * БЕЗ ПРОВОДА
 *   'osc save' сохраняет саму K («ручная поправка»). Она не следит за
 *   дрейфом, но и не хуже, чем двенадцать отдельных правок нулей.
 *
 * ФЛЕШ: пространство "spotty", ключи "oscref" (double, Гц) и "oscman" (float).
 * Чужие ключи ("hold", "lim") не трогаются.
 * ---------------------------------------------------------------------------
 */
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <esp_timer.h>
#include "spotty_config.h"

static_assert(JOINT_ZERO_US[SPOTTY_OSC_PROBE_CH] == 0,
              "Канал зонда генератора занят суставом: на него пойдёт меандр!");
static_assert(SPOTTY_OSC_PROBE_GPIO != SPOTTY_I2C_SDA && SPOTTY_OSC_PROBE_GPIO != SPOTTY_I2C_SCL,
              "GPIO зонда совпадает с I2C");
static_assert(SPOTTY_OSC_PROBE_GPIO != 0 && SPOTTY_OSC_PROBE_GPIO != 3 &&
              SPOTTY_OSC_PROBE_GPIO != 45 && SPOTTY_OSC_PROBE_GPIO != 46,
              "GPIO зонда — strapping-пин ESP32-S3, взять другой");

// Шаг ручной подстройки: 0.2 % = 3 мкс на 1500 = 0.54°.
static const float SPOTTY_OSC_NUDGE_PCT = 0.2f;
// Больше этого скачка при поданном ШИМ не применяем: все суставы дёрнутся разом.
static const float SPOTTY_OSC_LIVE_STEP = 0.006f;   // 0.6 % ≈ 1.6°

enum SpottyOscSrc : uint8_t { OSC_SRC_NONE, OSC_SRC_PROBE, OSC_SRC_MANUAL };

static double       spottyOscK      = 1.0;   // действующая поправка
static SpottyOscSrc spottyOscSrc    = OSC_SRC_NONE;
static double       spottyOscRefHz  = 0.0;   // эталон из флеша, 0 = нет
static double       spottyOscManK   = 0.0;   // ручная из флеша, 0 = нет
static double       spottyOscLastHz = 0.0;   // последний удачный замер, 0 = нет
static bool         spottyOscRefCfg = false; // эталон взят из spotty_config.h

// Фоновое слежение за генератором.
static const uint32_t SPOTTY_OSC_TRACK_MS   = 2000;     // интервал обновления
static const double   SPOTTY_OSC_TRACK_STEP = 0.0002;   // шаг K за обновление: 0.02 % ≈ 0.05°
static const double   SPOTTY_OSC_TRACK_NOTE = 0.001;    // печатать, если K ушла на 0.1 % ≈ 0.27°
static bool     spottyOscTrack    = true;    // 'osc track' выключает
static bool     spOscTrkHave      = false;   // есть прошлый снимок
static bool     spOscTrkLost      = false;   // зонд замолчал
static bool     spOscTrkEverOk    = false;   // хоть раз был сигнал (без провода — молчим)
static uint8_t  spOscTrkSilent    = 0;
static uint32_t spOscTrkMs        = 0;
static double   spOscTrkMinHz = 0, spOscTrkMaxHz = 0;
static double   spOscTrkNoteK = 0;           // K на момент последней печати

// ------------------------------- замер -------------------------------------
/*
 * ФОНОВЫЙ СЧЁТ (ревизия 2026-09-26, вечер). Канал зонда выдаёт меандр
 * ПОСТОЯННО, прерывание считает фронты и помнит время последнего. Частота —
 * по двум снимкам (число фронтов, время последнего фронта): оба конца
 * интервала — настоящие фронты, дробных периодов нет.
 * Нагрузка: ~330 коротких прерываний в секунду.
 */
static portMUX_TYPE      spOscMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t spOscN = 0;
static volatile int64_t  spOscT = 0;
static bool              spOscIsrOn = false;

static void IRAM_ATTR spottyOscIsr() {
  int64_t t = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&spOscMux);
  spOscN = spOscN + 1;
  spOscT = t;
  portEXIT_CRITICAL_ISR(&spOscMux);
}

struct SpOscSnap { uint32_t n; int64_t t; };
static SpOscSnap spOscSnap() {
  SpOscSnap s;
  portENTER_CRITICAL(&spOscMux);
  s.n = spOscN; s.t = spOscT;
  portEXIT_CRITICAL(&spOscMux);
  return s;
}
static double spOscFreq(const SpOscSnap &a, const SpOscSnap &b) {
  if (b.n < a.n + 3 || b.t <= a.t) return 0.0;
  return (double)(b.n - a.n) * 1e6 / (double)(b.t - a.t);
}

// Включить меандр на канале зонда и счёт. Повторный вызов безопасен:
// так же восстанавливается канал, погашенный чужим «погасить всё».
static void spottyOscProbeArm(Adafruit_PWMServoDriver &pca) {
  if (!spOscIsrOn) {
    pinMode(SPOTTY_OSC_PROBE_GPIO, INPUT_PULLDOWN);   // без провода — тишина
    attachInterrupt(digitalPinToInterrupt(SPOTTY_OSC_PROBE_GPIO), spottyOscIsr, RISING);
    spOscIsrOn = true;
  }
  pca.setPWM(SPOTTY_OSC_PROBE_CH, 0, 2048);           // меандр 50 %
}

struct SpottyOscMeas {
  bool     ok;
  double   oscHz;      // генератор, Гц
  double   pwmHz;      // кадр ШИМ, Гц
  double   spreadPpm;  // расхождение двух окон
  uint32_t edges;
  const char *why;     // причина отказа
};

// Пересчёт кадра ШИМ в частоту генератора с проверкой правдоподобия.
static bool spOscToOsc(double pwmHz, uint8_t pre, double &oscHz, const char *&why) {
  const double expect = (double)SPOTTY_OSC_HZ / (4096.0 * ((double)pre + 1.0));
  // Генератор PCA9685 по паспорту около 25 МГц, реальные экземпляры 23..28.
  // Шире ±15 % от расчётного — это не генератор, а помеха или не тот пин.
  if (fabs(pwmHz / expect - 1.0) > 0.15) { why = "частота вне ±15 % от расчётной — помеха или не тот канал"; return false; }
  oscHz = pwmHz * 4096.0 * ((double)pre + 1.0);
  return true;
}

/*
 * Явный замер, ~0.55 с, блокирующий. Суставы не трогает: пишется только
 * канал зонда. После замера меандр НЕ гасится — он нужен фоновому счёту.
 */
static SpottyOscMeas spottyOscMeasure(Adafruit_PWMServoDriver &pca) {
  SpottyOscMeas m = {false, 0, 0, 0, 0, ""};
  const uint8_t pre = pca.readPrescale();
  spottyOscProbeArm(pca);
  delay(30);                                          // пара кадров на установку
  SpOscSnap s0 = spOscSnap(); delay(250);
  SpOscSnap s1 = spOscSnap(); delay(250);
  SpOscSnap s2 = spOscSnap();
  m.edges = s2.n - s0.n;
  const double f1 = spOscFreq(s0, s1), f2 = spOscFreq(s1, s2);

  if (m.edges < 3) { m.why = "фронтов нет — провод зонда не подключён?"; return m; }
  if (f1 <= 0 || f2 <= 0) { m.why = "фронты рваные — плохой контакт?"; return m; }
  m.spreadPpm = fabs(f1 - f2) / ((f1 + f2) * 0.5) * 1e6;
  m.pwmHz = (f1 + f2) * 0.5;
  if (!spOscToOsc(m.pwmHz, pre, m.oscHz, m.why)) return m;
  if (m.spreadPpm > 500) { m.why = "два окна разошлись > 500 ppm — нестабильный сигнал"; return m; }
  m.ok = true;
  return m;
}

// ------------------------------- флеш --------------------------------------
static void spottyOscNvsLoad() {
  Preferences p;
  p.begin("spotty", true);
  spottyOscRefHz = p.getDouble("oscref", 0.0);
  spottyOscManK  = p.getFloat("oscman", 0.0f);
  p.end();
  // Флеш пуст — эталон из конфига (SPOTTY_OSC_REF_HZ, запись оператора).
  spottyOscRefCfg = false;
  if (spottyOscRefHz <= 0 && SPOTTY_OSC_REF_HZ > 0) {
    spottyOscRefHz = SPOTTY_OSC_REF_HZ;
    spottyOscRefCfg = true;
  }
}

// ---------------------------- применение -----------------------------------
static bool spottyOscSane(double k) { return fabs(k - 1.0) <= (double)SPOTTY_OSC_MAX_DEV; }

// Сменить K. При поданном ШИМ большой скачок запрещён: все суставы дёрнутся.
template <class Out>   // Print на ESP32, заглушка Serial на стенде hostcheck
static bool spottyOscApply(double k, SpottyOscSrc src, bool powered, Out &out) {
  if (!spottyOscSane(k)) {
    out.printf("[!] поправка %.5f вне ±%.0f %% — НЕ применена. Эталон испорчен или зонд врёт.\n",
               k, SPOTTY_OSC_MAX_DEV * 100.0f);
    return false;
  }
  if (powered && fabs(k - spottyOscK) > SPOTTY_OSC_LIVE_STEP) {
    out.printf("[!] скачок поправки %.2f %% при поданном ШИМ — суставы дёрнутся разом.\n",
               (k - spottyOscK) * 100.0);
    out.println("    Снять ШИМ и повторить, либо двигать шагами 'osc +' / 'osc -'.");
    return false;
  }
  spottyOscK = k; spottyOscSrc = src;
  return true;
}

template <class Out>   // Print на ESP32, заглушка Serial на стенде hostcheck
static void spottyOscStatus(Out &out) {
  const char *src = spottyOscSrc == OSC_SRC_PROBE  ? "по зонду (следит за дрейфом)"
                  : spottyOscSrc == OSC_SRC_MANUAL ? "ручная"
                                                   : "нет (K = 1, как раньше)";
  const double dUs = (spottyOscK - 1.0) * 1500.0;
  out.printf("генератор PCA9685: K = %.5f (%+.2f %%)  %s\n", spottyOscK, (spottyOscK - 1.0) * 100.0, src);
  out.printf("  на 1500 мкс это %+.1f мкс = %+.2f° по суставу (знак по JOINT_SIGN)\n",
             dUs, dUs / SPOTTY_US_PER_DEG);
  if (spottyOscLastHz > 0) out.printf("  замер: %.0f Гц", spottyOscLastHz);
  else                     out.print ("  замер: нет");
  if (spottyOscRefHz > 0)  out.printf("   эталон: %.0f Гц (%s)", spottyOscRefHz,
                                      spottyOscRefCfg ? "spotty_config.h" : "флеш");
  else                     out.print ("   эталон: не записан");
  if (spottyOscManK > 0)   out.printf("   ручная во флеше: %.5f", spottyOscManK);
  out.println();
  if (spottyOscLastHz > 0 && spottyOscRefHz > 0)
    out.printf("  уход от эталона: %+.0f ppm\n", (spottyOscLastHz / spottyOscRefHz - 1.0) * 1e6);
  out.printf("  фоновое слежение: %s", !spottyOscTrack ? "ВЫКЛ ('osc track')"
                                     : spOscTrkLost  ? "зонд МОЛЧИТ, K заморожена"
                                     : spottyOscSrc == OSC_SRC_PROBE ? "вкл, K ведётся по зонду"
                                     : "вкл, только замер (K ручная или эталона нет)");
  if (spOscTrkMinHz > 0) out.printf("; за сессию %.0f…%.0f Гц", spOscTrkMinHz, spOscTrkMaxHz);
  out.println();
}

/*
 * Старт. Вызывать после setPWMFreq и гашения выходов, ДО первой подачи ШИМ.
 * Порядок доверия: зонд + эталон > ручная из флеша > 1.0.
 */
template <class Out>   // Print на ESP32, заглушка Serial на стенде hostcheck
static void spottyOscBoot(Adafruit_PWMServoDriver &pca, Out &out) {
  spottyOscNvsLoad();
  out.println();
  out.printf("--- генератор PCA9685 (зонд: ch%u -> GPIO%u) ---\n",
             SPOTTY_OSC_PROBE_CH, SPOTTY_OSC_PROBE_GPIO);
  SpottyOscMeas m = spottyOscMeasure(pca);
  if (m.ok) {
    spottyOscLastHz = m.oscHz;
    out.printf("замер: генератор %.0f Гц, кадр %.3f Гц, окна сошлись до %.0f ppm\n",
               m.oscHz, m.pwmHz, m.spreadPpm);
  } else {
    out.printf("замер не удался: %s\n", m.why);
  }

  if (m.ok && spottyOscRefHz > 0) {
    if (!spottyOscApply(m.oscHz / spottyOscRefHz, OSC_SRC_PROBE, false, out)) {
      spottyOscK = 1.0; spottyOscSrc = OSC_SRC_NONE;
    }
  } else if (spottyOscManK > 0) {
    if (spottyOscRefHz > 0)
      out.println("[!] эталон есть, но зонд не ответил — беру РУЧНУЮ поправку, дрейф не отслеживается.");
    if (!spottyOscApply(spottyOscManK, OSC_SRC_MANUAL, false, out)) {
      spottyOscK = 1.0; spottyOscSrc = OSC_SRC_NONE;
    }
  } else {
    spottyOscK = 1.0; spottyOscSrc = OSC_SRC_NONE;
    if (spottyOscRefHz > 0)
      out.println("[!] эталон есть, но зонд не ответил и ручной поправки нет — K = 1.");
    else if (m.ok)
      out.println("эталон не записан. Выставить ноги ('osc +'/'osc -'), затем 'osc save'.");
  }
  spottyOscStatus(out);
}

/*
 * ФОНОВОЕ СЛЕЖЕНИЕ. Звать из loop() каждый проход; работает раз в
 * SPOTTY_OSC_TRACK_MS. Возвращает true, если K изменилась — скетчам без
 * покадрового пересчёта (servo_hold, servo_calib) надо переписать каналы.
 *
 * K ведётся ТОЛЬКО при источнике «зонд» (эталон есть). Ручную K, которую
 * оператор сейчас подбирает, фон не трогает — только меряет.
 * Шаг ограничен SPOTTY_OSC_TRACK_STEP: даже если генератор прыгнет,
 * суставы поедут на 0.05° за 2 с, а не рывком.
 */
template <class Out>
static bool spottyOscTrackTick(Adafruit_PWMServoDriver &pca, Out &out) {
  if (!spottyOscTrack || !spOscIsrOn) return false;
  const uint32_t ms = millis();
  if (ms - spOscTrkMs < SPOTTY_OSC_TRACK_MS) return false;
  spOscTrkMs = ms;

  static SpOscSnap prev;
  SpOscSnap cur = spOscSnap();
  const bool had = spOscTrkHave;
  const SpOscSnap a = prev;
  prev = cur; spOscTrkHave = true;
  if (!had) return false;

  if (cur.n - a.n < 3) {                         // тишина: канал погасили или провод отошёл
    if (++spOscTrkSilent == 1) spottyOscProbeArm(pca);   // вернуть меандр
    if (spOscTrkSilent == 2 && !spOscTrkLost && spOscTrkEverOk) {
      spOscTrkLost = true;
      out.println("[osc] зонд молчит — K заморожена на последнем значении.");
    }
    return false;
  }
  spOscTrkSilent = 0;
  if (spOscTrkLost) { spOscTrkLost = false; out.println("[osc] зонд снова отвечает."); }

  double oscHz = 0; const char *why = "";
  if (!spOscToOsc(spOscFreq(a, cur), pca.readPrescale(), oscHz, why)) return false;
  spottyOscLastHz = oscHz;
  spOscTrkEverOk = true;
  if (spOscTrkMinHz == 0 || oscHz < spOscTrkMinHz) spOscTrkMinHz = oscHz;
  if (oscHz > spOscTrkMaxHz) spOscTrkMaxHz = oscHz;

  if (spottyOscSrc != OSC_SRC_PROBE || spottyOscRefHz <= 0) return false;
  const double target = oscHz / spottyOscRefHz;
  if (!spottyOscSane(target)) return false;
  double d = target - spottyOscK;
  if (d >  SPOTTY_OSC_TRACK_STEP) d =  SPOTTY_OSC_TRACK_STEP;
  if (d < -SPOTTY_OSC_TRACK_STEP) d = -SPOTTY_OSC_TRACK_STEP;
  if (fabs(d) < 1e-6) return false;               // < 1 ppm — не трогать
  spottyOscK += d;
  if (spOscTrkNoteK == 0) spOscTrkNoteK = spottyOscK;
  if (fabs(spottyOscK - spOscTrkNoteK) >= SPOTTY_OSC_TRACK_NOTE) {
    out.printf("[osc] генератор %.0f Гц, K %.5f (%+.2f° на 1500 мкс с прошлой записи)\n",
               oscHz, spottyOscK, (spottyOscK - spOscTrkNoteK) * 1500.0 / SPOTTY_US_PER_DEG);
    spOscTrkNoteK = spottyOscK;
  }
  return true;
}

template <class Out>   // Print на ESP32, заглушка Serial на стенде hostcheck
static void spottyOscHelp(Out &out) {
  out.println("  --- генератор PCA9685 ---");
  out.println("  osc          замерить генератор, обновить поправку, показать");
  out.println("  osc + [%]    удлинить ВСЕ импульсы (шаг 0.2 % = 3 мкс = 0.5°)");
  out.println("  osc - [%]    укоротить ВСЕ импульсы");
  out.println("  osc = <K>    задать поправку числом (0.95..1.05)");
  out.println("  osc save     ноги стоят ровно -> запомнить (с зондом: эталон частоты)");
  out.println("  osc clr      стереть эталон и ручную поправку, K = 1");
  out.println("  osc track    вкл/выкл фоновое слежение (по умолчанию ВКЛ, шаг 0.02 %/2 с)");
}

/*
 * Разбор аргументов после слова 'osc'. Возвращает true, если K изменилась —
 * вызывающий скетч должен тогда переписать удерживаемые каналы.
 * powered — подан ли сейчас ШИМ на суставы.
 */
template <class Out>   // Print на ESP32, заглушка Serial на стенде hostcheck
static bool spottyOscCommand(const char *a, Adafruit_PWMServoDriver &pca, bool powered, Out &out) {
  while (*a == ' ') a++;

  if (!*a) {                                        // замер
    SpottyOscMeas m = spottyOscMeasure(pca);
    bool changed = false;
    if (m.ok) {
      spottyOscLastHz = m.oscHz;
      out.printf("замер: генератор %.0f Гц, кадр %.3f Гц, окна сошлись до %.0f ppm\n",
                 m.oscHz, m.pwmHz, m.spreadPpm);
      if (spottyOscRefHz > 0) {
        double k = m.oscHz / spottyOscRefHz;
        if (spottyOscSrc == OSC_SRC_MANUAL && spottyOscManK <= 0)
          out.println("[i] несохранённая ручная поправка заменена расчётом по эталону");
        changed = spottyOscApply(k, OSC_SRC_PROBE, powered, out);
      }
    } else {
      out.printf("замер не удался: %s\n", m.why);
    }
    spottyOscStatus(out);
    return changed;
  }

  if (*a == '+' || *a == '-') {
    const float sgn = (*a == '+') ? 1.0f : -1.0f;
    float pct = atof(a + 1);
    if (pct <= 0.0f) pct = SPOTTY_OSC_NUDGE_PCT;
    if (pct > 0.6f) { out.println("шаг не больше 0.6 %"); return false; }
    bool ok = spottyOscApply(spottyOscK * (1.0 + sgn * pct / 100.0), OSC_SRC_MANUAL, powered, out);
    spottyOscStatus(out);
    if (ok) out.println("  не сохранено: 'osc save', когда ноги встанут ровно");
    return ok;
  }

  if (*a == '=') {
    double k = atof(a + 1);
    if (k <= 0) { out.println("osc = <K>, например osc = 1.012"); return false; }
    bool ok = spottyOscApply(k, OSC_SRC_MANUAL, powered, out);
    spottyOscStatus(out);
    return ok;
  }

  if (!strncmp(a, "track", 5)) {
    spottyOscTrack = !spottyOscTrack;
    if (spottyOscTrack) spottyOscProbeArm(pca);
    spOscTrkHave = false;                           // начать с чистого снимка
    out.printf("фоновое слежение %s\n", spottyOscTrack ? "ВКЛ" : "ВЫКЛ — K заморожена");
    return false;
  }

  if (!strncmp(a, "save", 4)) {
    Preferences p;
    SpottyOscMeas m = spottyOscMeasure(pca);
    p.begin("spotty", false);
    if (m.ok) {
      spottyOscLastHz = m.oscHz;
      spottyOscRefHz  = m.oscHz / spottyOscK;
      spottyOscRefCfg = false;
      p.putDouble("oscref", spottyOscRefHz);
      p.remove("oscman");
      spottyOscManK = 0;
      spottyOscSrc = OSC_SRC_PROBE;
      out.printf("ЭТАЛОН ЗАПИСАН: %.0f Гц (сейчас %.0f Гц, K = %.5f).\n",
                 spottyOscRefHz, m.oscHz, spottyOscK);
      out.println("Дальше при каждом старте поправка считается по зонду сама.");
    } else {
      spottyOscManK = spottyOscK;
      p.putFloat("oscman", (float)spottyOscK);
      out.printf("зонд не ответил (%s)\n", m.why);
      out.printf("записана РУЧНАЯ поправка K = %.5f. Дрейф она не отслеживает.\n", spottyOscK);
    }
    p.end();
    return false;                                   // K не менялась
  }

  if (!strncmp(a, "clr", 3)) {
    if (!spottyOscApply(1.0, OSC_SRC_NONE, powered, out)) return false;
    Preferences p;
    p.begin("spotty", false);
    p.remove("oscref"); p.remove("oscman");
    p.end();
    spottyOscRefHz = 0; spottyOscManK = 0; spottyOscRefCfg = false;
    out.println("флеш очищен, K = 1 до перезагрузки.");
    if (SPOTTY_OSC_REF_HZ > 0)
      out.printf("После перезагрузки снова возьмётся эталон из spotty_config.h (%.0f Гц).\n",
                 SPOTTY_OSC_REF_HZ);
    return true;
  }

  spottyOscHelp(out);
  return false;
}

// ---------------------------------------------------------------------------
// Замена pca.writeMicroseconds() для servo_hold и servo_calib.
// Формула библиотеки Adafruit 3.0.2 буква в букву (включая чтение делителя
// и усечение double -> uint16_t), плюс умножение на K в самом конце.
// При K = 1.0 результат побитово равен writeMicroseconds().
// ---------------------------------------------------------------------------
static void spottyWriteUs(Adafruit_PWMServoDriver &pca, uint8_t ch, uint16_t us) {
  double pulse = us;
  double pulselength = 1000000;
  uint16_t prescale = pca.readPrescale();
  prescale += 1;
  pulselength *= prescale;
  pulselength /= pca.getOscillatorFrequency();
  pulse /= pulselength;
  pulse *= spottyOscK;
  pca.setPWM(ch, 0, (uint16_t)pulse);
}
