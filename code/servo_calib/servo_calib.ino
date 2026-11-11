/*
 * servo_calib.ino — калибровка «импульс <-> угол» по транспортиру
 * Проект Spotty (OpenQuadruped).  Фаза: CALIBRATION.md, п.4, шаги 4–6.
 * ---------------------------------------------------------------------------
 * ОТДЕЛЬНЫЙ скетч, а не режим servo_test — намеренно.
 * SAFETY.md, правило 4: пока соответствие канал -> сустав не проверено,
 * за раз двигается ровно ОДИН канал. Здесь физически нет команды, способной
 * подать ШИМ более чем на один канал. Это свойство двоичного файла, а не
 * дисциплины оператора.
 *
 * СТЕНД (заявление оператора, 2026-08-20):
 *   серво лежат на столе поодиночке, качалка надета и служит стрелкой,
 *   угол снимается транспортиром. Механической нагрузки нет.
 *
 * ПАСПОРТ QY3240MG (два независимых магазинных листинга, НЕ даташит завода):
 *   напряжение   5.0 ... 7.4 В        <- ПОТОЛОК 7.4 В
 *   нейтраль     1520 мкс            <- не 1500
 *   рабочая част.~330 Гц
 *   полный ход   500 ... 2500 мкс = 180°  (~11.1 мкс на градус)
 *   мёртвая зона 3 мкс
 * Листинги расходятся в типе мотора и весе — то есть это не заводской
 * документ. Электрически важные числа в обоих совпадают.
 *
 * РАЗРЕШЕНИЕ ПО ИМПУЛЬСУ (важно для точности калибровки):
 *   PCA9685 делит период на 4096 шагов.
 *     50 Гц  -> 20000/4096 = 4.88 мкс на шаг   ХУЖЕ мёртвой зоны 3 мкс
 *    330 Гц  ->  3030/4096 = 0.74 мкс на шаг   мельче мёртвой зоны
 *   Поэтому по умолчанию здесь 330 Гц, а не историческое 50 Гц.
 *   Частота пишется в отчёт: калибровка без указания частоты бессмысленна.
 *
 * БЕЗОПАСНОСТЬ НА СТЕНДЕ БЕЗ ЛАБОРАТОРНОГО БП
 *   Токового датчика в железе нет (HARDWARE.md п.3) — стойло НЕ детектируется
 *   ни программно, ни косвенно. Отсюда два механизма:
 *     1) рабочее окно (win) — мягкий кламп, по умолчанию узкий, вокруг 1520;
 *        расширяется только явной командой оператора;
 *     2) 'pr' — «клевок»: импульс подаётся, удерживается заданное время и
 *        СНИМАЕТСЯ. Края диапазона ищутся только так, чтобы упор в
 *        механический ограничитель не мог длиться дольше клевка.
 *   Разрыв питания серво остаётся единственным аварийным остановом.
 *
 * ПОРЯДОК РАБОТЫ С ОДНИМ СЕРВО
 *   ch 3            выбрать канал (все прочие обесточены)
 *   g 1520          встать в паспортную нейтраль, надеть качалку по нулю
 *   p 90            записать точку: текущий импульс = 90° по транспортиру
 *   g 1300 / p 74   ещё точка... нужно минимум 3, лучше 5-7 по всему ходу
 *   fit             МНК: мкс на градус, импульс 90°, максимальная невязка
 *   pr 1150         клевком нащупать нижний край, затем lo
 *   pr 1900         то же сверху, затем hi
 *   sv              сохранить канал
 *   tab             строки для таблицы CALIBRATION.md, раздел 2
 *
 * БИБЛИОТЕКА: Adafruit PWM Servo Driver Library 3.0.2
 * ПОРТ: Serial 115200. Команда исполняется по '\n', '\r' или по паузе.
 * ---------------------------------------------------------------------------
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include "spotty_config.h"   // ЕДИНСТВЕННЫЙ источник нулей, пределов и знаков
#include "spotty_osc.h"      // поправка на уход генератора PCA9685 (2026-09-26)

static const uint8_t  I2C_SDA_PIN = 8;
static const uint8_t  I2C_SCL_PIN = 9;
static const uint8_t  PCA_ADDR    = 0x40;
static const uint32_t I2C_HZ      = 400000UL;

// Абсолютный предел. Мягкое окно (win) всегда внутри него.
// Измерено 2026-08-20: ход 500..2500 мкс = 0..355 град, отклик повторяем
// на всём диапазоне (две пробы на 2500 дали 354 и 355 град).
// Участок 2400..2500 линеен хуже: 6.25 мкс/град против рабочих 5.54.
// Калибровку вести НИЖЕ 2400.
static const uint16_t ABS_MIN_US = 500;
static const uint16_t ABS_MAX_US = 2500;
static const uint16_t SPEC_NEUTRAL_US = 1520;

Adafruit_PWMServoDriver pca(PCA_ADDR);

// ------------------------------ состояние ----------------------------------
static int8_t   curCh    = -1;      // выбранный канал, -1 = не выбран
static bool     powered  = false;   // подан ли ШИМ на curCh
static uint16_t curUs    = SPEC_NEUTRAL_US;
static uint16_t winLo    = 1220;    // мягкое окно, консервативное
static uint16_t winHi    = 1820;
static uint16_t stepUs   = 10;
static uint16_t probeMs  = 500;     // длительность клевка
static uint16_t slewUsS  = 400;     // скорость перехода, мкс/с
static float    pwmHz    = 330.0f;
static uint32_t vBatMv   = 0;       // измеренное напряжение серво, мВ
static char     whoStr[24] = "";
static char     dateStr[16] = "";

// точки текущего канала
static const uint8_t MAX_PTS = 16;
static uint16_t ptUs[MAX_PTS];
static float    ptDeg[MAX_PTS];
static uint8_t  nPts = 0;

struct Cal {
  bool     done;
  uint16_t zero;      // импульс нулевой позы сустава (по приспособлению)
  float    usPerDeg;
  float    us90;
  float    maxResid;
  uint16_t lo, hi;
  uint8_t  n;
  float    hz;
  uint32_t mv;
};
static Cal cal[16];

/* ---------------------------------------------------------------------------
 * ИСТОРИЧЕСКАЯ ТАБЛИЦА ЗАМЕРА 2026-08-20. БОЛЬШЕ НЕ ИСТОЧНИК ПРАВДЫ.
 *
 * РЕВИЗИЯ 2026-09-12. До неё этот скетч жил по своей копии нулей и не знал
 * про spotty_config.h вовсе. Копий набралось три — конфиг, локальная таблица
 * servo_hold и эта, — и они разошлись на семи каналах из двенадцати, на
 * ch9 (FR хип) сразу на 40 мкс = 7.2°. То есть «перезалил calib» означало
 * «поработал по другим нулям», и заметить это было нечем: сверки не было.
 *
 * РЕШЕНИЕ ОПЕРАТОРА 2026-09-12: истиной считается ПОСЛЕДНЯЯ правка таблицы
 * нулей, то есть spotty_config.h. Теперь рабочая таблица берётся оттуда, а
 * эта остаётся в исходнике как исторический протокол и печатается сверкой:
 * расхождение видно сразу, числом и в градусах.
 *
 * Что здесь ценно и после ревизии: ПРЕДЕЛЫ. Они снимались подводом до
 * касания и совпадают с конфигом по всем двенадцати каналам — расходятся
 * только нули. Это и есть M9: где именно стоит сустав, когда ему подан его
 * нуль. Пока M9_ПРОТОКОЛ.md не заполнен, «правильную» таблицу из трёх
 * назвать нечем — только решением оператора, которое записано выше.
 *
 * Дата снятия : 2026-08-20
 * Метод       : подвод малым шагом до касания, отступ 3° внутрь
 * Питание     : 2S LiPo, 7.0…8.0 В, нестабильно
 * Масштаб     : 5.57 мкс/градус
 * ---------------------------------------------------------------------------
 */
struct Factory { uint8_t ch; uint16_t zero, lo, hi; const char *note; };
static const Factory FACTORY[] = {
  // Разводка и пределы установлены физически 2026-08-20. Каналы сгруппированы
  // ПО ТИПУ СУСТАВА, не по ноге. Диагональные пары дают в сумме 14.
  {  0, 1520,  700, 1780, "BL задняя левая, колено" },
  {  1, 1440, 1200, 2250, "BR задняя правая, колено" },
  {  2, 1460, 1080, 1980, "BL задняя левая, бедро" },
  {  4, 1445,  960, 1820, "BR задняя правая, бедро" },
  {  5, 1485, 1350, 1780, "BL задняя левая, хип" },
  {  6, 1430, 1130, 1550, "BR задняя правая, хип" },
  {  8, 1430, 1140, 1560, "FL передняя левая, хип" },
  {  9, 1460, 1350, 1780, "FR передняя правая, хип" },
  { 10, 1470, 1010, 1990, "FL передняя левая, бедро" },
  { 12, 1540, 1000, 1890, "FR передняя правая, бедро" },
  { 13, 1520,  710, 1730, "FL передняя левая, колено" },
  { 14, 1490, 1220, 2290, "FR передняя правая, колено" },
};
static const uint8_t FACTORY_N = sizeof(FACTORY) / sizeof(FACTORY[0]);

// Имя канала для печати. Единственное, ради чего осталась FACTORY[].
static const char *chNote(uint8_t ch) {
  for (uint8_t i = 0; i < FACTORY_N; i++)
    if (FACTORY[i].ch == ch) return FACTORY[i].note;
  return "-";
}

// Рабочая таблица из ЕДИНСТВЕННОГО источника правды — spotty_config.h.
static void applyConfig(bool verbose) {
  uint8_t n = 0;
  for (uint8_t ch = 0; ch < 16; ch++) {
    if (!spottyChUsed(ch)) { cal[ch].zero = cal[ch].lo = cal[ch].hi = 0; continue; }
    cal[ch].zero = JOINT_ZERO_US[ch];
    cal[ch].lo   = JOINT_MIN_US[ch];
    cal[ch].hi   = JOINT_MAX_US[ch];
    n++;
    if (verbose)
      Serial.printf("  ch%-3u нуль %u  предел %u..%u  (%s)\n",
                    ch, cal[ch].zero, cal[ch].lo, cal[ch].hi, chNote(ch));
  }
  if (verbose)
    Serial.printf("Таблица взята из spotty_config.h: %u каналов. 'hist' — исторический замер.\n", n);
}

// Сверка исторического замера с конфигом. Молчать об этом нельзя: разница
// между этими двумя таблицами и есть неснятый M9, канал за каналом.
static void histCompare() {
  Serial.println(F("\n--- исторический замер 2026-08-20 против spotty_config.h ---"));
  Serial.println(F("  ch   истор  конфиг   разница        сустав"));
  uint8_t diff = 0;
  float sum = 0;
  for (uint8_t i = 0; i < FACTORY_N; i++) {
    const Factory &f = FACTORY[i];
    if (!spottyChUsed(f.ch)) {
      Serial.printf("  %-4u %-6u   --     канала нет в конфиге\n", f.ch, f.zero);
      diff++; continue;
    }
    int d = (int)f.zero - (int)JOINT_ZERO_US[f.ch];
    if (f.lo != JOINT_MIN_US[f.ch] || f.hi != JOINT_MAX_US[f.ch])
      Serial.printf("  %-4u ПРЕДЕЛ истор %u..%u против конфига %u..%u\n",
                    f.ch, f.lo, f.hi, JOINT_MIN_US[f.ch], JOINT_MAX_US[f.ch]);
    if (!d) continue;
    diff++; sum += (d < 0 ? -d : d) / SPOTTY_US_PER_DEG;
    Serial.printf("  %-4u %-6u %-6u %+5d мкс = %+5.1f°   %s\n",
                  f.ch, f.zero, JOINT_ZERO_US[f.ch], d, d / SPOTTY_US_PER_DEG,
                  chNote(f.ch));
  }
  if (!diff) { Serial.println(F("  расхождений нет")); return; }
  Serial.printf("расхождений по нулям: %u, сумма модулей %.1f°\n", diff, sum);
  Serial.println(F("Рабочая таблица — КОНФИГ (решение оператора 2026-09-12)."));
  Serial.println(F("Какая из двух ближе к правде, решает только M9 транспортиром."));
}

// Пределы, найденные при сборке, живут во флеше: сессия сборки длинная,
// сброс питания посреди неё не должен стирать часы работы.
static Preferences prefs;
static float marginDeg = 3.0f;      // отступ от точки касания, градусов

struct Persist { uint16_t zero, lo, hi; };

// ------------------------------ приём строки -------------------------------
static char     line[80];
static uint8_t  lineLen = 0;
static uint32_t rxTotal = 0, lastRxMs = 0, lastBeat = 0;
static bool     echoOn = true;
static const uint32_t LINE_IDLE_MS = 120;
static const uint32_t BEAT_MS      = 8000;

// ------------------------------ утилиты ------------------------------------
// Эффективный предел = пересечение мягкого окна и найденных при сборке
// пределов ЭТОГО канала. Найденный предел сильнее окна: он получен из
// механики, а окно — из свойств серво.
static uint16_t clampWin(int32_t us) {
  int32_t lo = winLo, hi = winHi;
  if (curCh >= 0) {
    if (cal[curCh].lo && cal[curCh].lo > lo) lo = cal[curCh].lo;
    if (cal[curCh].hi && cal[curCh].hi < hi) hi = cal[curCh].hi;
  }
  if (us < lo) return (uint16_t)lo;
  if (us > hi) return (uint16_t)hi;
  return (uint16_t)us;
}

static void nvsSave() {
  Persist a[16];
  for (uint8_t i = 0; i < 16; i++) { a[i].zero = cal[i].zero; a[i].lo = cal[i].lo; a[i].hi = cal[i].hi; }
  prefs.begin("spotty", false);
  prefs.putBytes("lim", a, sizeof(a));
  prefs.end();
  Serial.println(F("пределы и нули записаны во флеш"));
}

static void nvsLoad(bool quiet) {
  Persist a[16];
  prefs.begin("spotty", true);
  size_t n = prefs.getBytes("lim", a, sizeof(a));
  prefs.end();
  if (n != sizeof(a)) { if (!quiet) Serial.println(F("во флеше ничего нет")); return; }
  // РЕВИЗИЯ 2026-09-12. Было: цикл копировал ВСЕ шестнадцать записей, включая
  // пустые. Если во флеше лежали данные по трём каналам, остальные девять
  // обнулялись — рабочая таблица уничтожалась молча, и 'exp' после этого
  // выдал бы девять нулей. Теперь пустая запись пропускается.
  uint8_t cnt = 0, over = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (!a[i].zero && !a[i].lo && !a[i].hi) continue;      // во флеше пусто
    // Флеш главнее конфига по прямому назначению скетча — он держит замер
    // незаконченной сессии. Но оператору обязаны сказать, что перекрыто.
    if (spottyChUsed(i) && a[i].zero && a[i].zero != JOINT_ZERO_US[i]) {
      int d = (int)a[i].zero - (int)JOINT_ZERO_US[i];
      Serial.printf("  ch%-2u флеш %u перекрывает нуль конфига %u (%+d мкс = %+.1f°)\n",
                    i, a[i].zero, JOINT_ZERO_US[i], d, d / SPOTTY_US_PER_DEG);
      over++;
    }
    cal[i].zero = a[i].zero; cal[i].lo = a[i].lo; cal[i].hi = a[i].hi;
    cnt++;
  }
  Serial.printf("из флеша прочитано каналов: %u, из них перекрывают конфиг: %u\n",
                cnt, over);
  if (over) Serial.println(F("  ('er' сотрёт флеш и вернёт нули из spotty_config.h)"));
}

static void printLimits() {
  Serial.println(F("\n--- нули и пределы по каналам ---"));
  Serial.println(F("  ch   нуль    мин    макс   ход, град"));
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < 16; i++) {
    Cal &e = cal[i];
    if (!e.zero && !e.lo && !e.hi) continue;
    float span = (e.lo && e.hi) ? (e.hi - e.lo) / 5.57f : 0;
    Serial.printf("  %-4u %-7u %-6u %-6u %.1f\n", i,
                  e.zero, e.lo, e.hi, span);
    cnt++;
  }
  if (!cnt) Serial.println(F("  (пусто)"));
}

// Готовые массивы для вставки в прошивку.
static void exportArrays() {
  Serial.println(F("\n// ---- вставить в прошивку ----"));
  Serial.println(F("// Пределы получены физически при сборке: серво подводилось"));
  Serial.println(F("// малым шагом до касания, затем отступ внутрь на marginDeg."));
  Serial.printf ("// Отступ при съёме: %.1f град = %.0f мкс\n", marginDeg, marginDeg * 5.57f);
  const char *nm[3] = {"JOINT_ZERO_US", "JOINT_MIN_US", "JOINT_MAX_US"};
  for (uint8_t f = 0; f < 3; f++) {
    Serial.printf("static const uint16_t %s[16] = {", nm[f]);
    for (uint8_t i = 0; i < 16; i++) {
      uint16_t v = f == 0 ? cal[i].zero : f == 1 ? cal[i].lo : cal[i].hi;
      Serial.printf("%u%s", v, i == 15 ? "" : ", ");
    }
    Serial.println(F("};"));
  }
  Serial.println(F("// 0 = канал не откалиброван, движение по нему запрещать"));
}

static void chOff(uint8_t ch) { pca.setPWM(ch, 0, 4096); }   // full-off

static void allOff() {
  for (uint8_t ch = 0; ch < 16; ch++) if (ch != SPOTTY_OSC_PROBE_CH) chOff(ch);   // канал зонда генератора не трогаем: на нём меандр фонового слежения (spotty_osc.h)
  powered = false;
}

// Плавный переход. Резкий скачок на неизвестном серво — лишний удар по
// редуктору и лишний бросок тока; на стенде это ничего не стоит смягчить.
static void slewTo(uint16_t target) {
  if (curCh < 0) { Serial.println(F("канал не выбран: 'ch <0..15>'")); return; }
  target = clampWin(target);
  if (!powered) {
    // Серво обесточено и физически стоит неизвестно где. Первая подача ШИМ —
    // всегда скачок в командную точку, плавно его сделать нечем: обратной
    // связи по положению нет. Поэтому первый выход делается в нейтраль.
    curUs = target;
    Serial.println(F("[!] первая подача ШИМ на канал: серво прыгнет в точку"));
  }
  int32_t from = curUs, to = target;
  int32_t diff = to - from;
  uint32_t ms = (uint32_t)((abs(diff) * 1000.0f) / (float)slewUsS);
  if (ms < 20) ms = 20;
  uint32_t t0 = millis(), el;
  while ((el = millis() - t0) < ms) {
    int32_t us = from + (diff * (int32_t)el) / (int32_t)ms;
    spottyWriteUs(pca, (uint8_t)curCh, (uint16_t)us);
    delay(5);
  }
  spottyWriteUs(pca, (uint8_t)curCh, (uint16_t)to);
  curUs = (uint16_t)to;
  powered = true;
}

static void releaseCh() {
  if (curCh >= 0) chOff((uint8_t)curCh);
  powered = false;
  Serial.println(F("ШИМ снят, серво отпущено"));
}

// ------------------------------- МНК ---------------------------------------
// deg = k*us + c. Возвращает false при вырожденных данных.
static bool fitLine(float &k, float &c, float &maxResid) {
  if (nPts < 2) return false;
  float sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (uint8_t i = 0; i < nPts; i++) {
    float x = ptUs[i], y = ptDeg[i];
    sx += x; sy += y; sxx += x * x; sxy += x * y;
  }
  float n = nPts;
  float den = n * sxx - sx * sx;
  if (den == 0.0f) return false;
  k = (n * sxy - sx * sy) / den;
  c = (sy - k * sx) / n;
  if (k == 0.0f) return false;
  maxResid = 0;
  for (uint8_t i = 0; i < nPts; i++) {
    float r = ptDeg[i] - (k * ptUs[i] + c);
    if (r < 0) r = -r;
    if (r > maxResid) maxResid = r;
  }
  return true;
}

static void doFit(bool quiet) {
  float k, c, mr;
  if (!fitLine(k, c, mr)) {
    Serial.println(F("нужно минимум 2 точки в разных положениях: 'p <угол>'"));
    return;
  }
  float usPerDeg = 1.0f / k;
  float us90     = (90.0f - c) / k;
  if (!quiet) {
    Serial.println(F("--- результат МНК ---"));
    Serial.printf("точек            : %u\n", nPts);
    Serial.printf("мкс на градус    : %.3f  (паспортное ~11.1)\n",
                  usPerDeg < 0 ? -usPerDeg : usPerDeg);
    Serial.printf("направление      : %s\n",
                  usPerDeg > 0 ? "прямое (больше мкс = больше угол)"
                               : "ИНВЕРСИЯ (больше мкс = меньше угол)");
    Serial.printf("импульс при 90   : %.1f мкс  (паспортная нейтраль %u)\n",
                  us90, SPEC_NEUTRAL_US);
    Serial.printf("макс. невязка    : %.2f град\n", mr);
    if (mr > 3.0f)
      Serial.println(F("[!] невязка велика: люфт качалки, перекос транспортира\n"
                       "    или серво нелинейно у краёв. Добавь точек в середине."));
    if (nPts < 3)
      Serial.println(F("[!] две точки дают прямую всегда. Невязка станет\n"
                       "    осмысленной с третьей точки."));
  }
}

// ---------------------------- команды --------------------------------------
static void printPts() {
  if (!nPts) { Serial.println(F("точек нет")); return; }
  Serial.println(F("  #   импульс, мкс   угол, град"));
  for (uint8_t i = 0; i < nPts; i++)
    Serial.printf("  %-3u %-14u %.1f\n", i + 1, ptUs[i], ptDeg[i]);
}

static void saveCh() {
  if (curCh < 0) { Serial.println(F("канал не выбран")); return; }
  float k, c, mr;
  if (!fitLine(k, c, mr)) { Serial.println(F("нечего сохранять: мало точек")); return; }
  Cal &e = cal[curCh];
  e.done     = true;
  e.usPerDeg = 1.0f / k;
  e.us90     = (90.0f - c) / k;
  e.maxResid = mr;
  // Края, помеченные командами lo/hi, приоритетнее окна. cal[] статичен и
  // обнулён, поэтому 0 надёжно означает "оператор край не отмечал".
  if (!e.lo) e.lo = winLo;
  if (!e.hi) e.hi = winHi;
  e.n  = nPts;
  e.hz = pwmHz;
  e.mv = vBatMv;
  Serial.printf("канал %d сохранён\n", curCh);
  if (!vBatMv)
    Serial.println(F("[!] напряжение питания серво не задано: 'vb <мВ>'.\n"
                     "    CALIBRATION.md п.2 требует его в каждой строке."));
}

static void printTable() {
  Serial.println();
  Serial.println(F("Вставить в CALIBRATION.md, раздел 2 (Measured values):"));
  Serial.println(F("| Дата | Канал PCA9685 | Нога | Сустав | Нейтраль, мкс | Мин, мкс | Макс, мкс | Направление | Метод измерения | Кто измерял |"));
  Serial.println(F("|---|---|---|---|---|---|---|---|---|---|"));
  uint8_t cnt = 0;
  for (uint8_t ch = 0; ch < 16; ch++) {
    if (!cal[ch].done) continue;
    Cal &e = cal[ch];
    Serial.printf("| %s | %u | — | — | %.0f | %u | %u | %s | транспортир, качалка-стрелка, стенд; %.0f Гц; %.2f В; %u точек; невязка %.2f° | %s |\n",
                  dateStr[0] ? dateStr : "—",
                  ch, e.us90, e.lo, e.hi,
                  e.usPerDeg > 0 ? "прямое" : "инверсия",
                  e.hz, e.mv / 1000.0f, e.n, e.maxResid,
                  whoStr[0] ? whoStr : "—");
    cnt++;
  }
  if (!cnt) Serial.println(F("(пусто — ни один канал не сохранён)"));
  Serial.println(F("\nНога и сустав оставлены прочерками намеренно: на стенде\n"
                   "серво не привязано к суставу. Заполнять при сборке."));
  Serial.println(F("Отдельно, для прошивки — мкс на градус по каналам:"));
  for (uint8_t ch = 0; ch < 16; ch++)
    if (cal[ch].done)
      Serial.printf("  ch%-3u  %.3f мкс/град   90° = %.1f мкс\n",
                    ch, cal[ch].usPerDeg, cal[ch].us90);
}

static void status() {
  Serial.println(F("--- состояние ---"));
  spottyOscStatus(Serial);
  if (curCh < 0) Serial.println(F("канал      : НЕ ВЫБРАН"));
  else Serial.printf("канал      : %d  (%s)\n", curCh, powered ? "под ШИМ" : "обесточен");
  Serial.printf("импульс    : %u мкс\n", curUs);
  Serial.printf("окно       : %u .. %u мкс  (абсолютный предел %u..%u)\n",
                winLo, winHi, ABS_MIN_US, ABS_MAX_US);
  Serial.printf("шаг        : %u мкс   клевок %u мс   скорость %u мкс/с\n",
                stepUs, probeMs, slewUsS);
  Serial.printf("частота    : %.0f Гц  ->  %.2f мкс на шаг PCA9685\n",
                pwmHz, (1000000.0f / pwmHz) / 4096.0f);
  if (!vBatMv) Serial.println(F("питание    : НЕ ЗАДАНО ('vb <мВ>')"));
  else Serial.printf("питание    : %.2f В%s\n", vBatMv / 1000.0f,
                     vBatMv > 7400 ? "   <- ВЫШЕ ПАСПОРТНЫХ 7.4 В" : "");
  Serial.printf("точек      : %u\n", nPts);
}

static void help() {
  Serial.println(F(
    "\n=== servo_calib — команды ===\n"
    "  ВЫБОР И ДВИЖЕНИЕ (всегда ровно один канал)\n"
    "  ch <0..15>   выбрать канал; все остальные обесточиваются\n"
    "  g <us>       плавно перейти на импульс и УДЕРЖИВАТЬ\n"
    "  +            шаг вверх на stepUs\n"
    "  -            шаг вниз на stepUs\n"
    "  rel          снять ШИМ (отпустить серво)\n"
    "  pr <us>      КЛЕВОК: подать, подержать probeMs и снять.\n"
    "               Только так ищутся края диапазона.\n"
    "  x            обесточить всё\n"
    "\n  ИЗМЕРЕНИЕ\n"
    "  p <град>     записать точку: текущий импульс = этот угол\n"
    "  pts          показать точки\n"
    "  dp           удалить последнюю точку\n"
    "  clr          стереть все точки текущего канала\n"
    "  fit          МНК: мкс/градус, импульс 90 град, невязка\n"
    "  z            записать текущий импульс как НУЛЬ сустава (по приспособлению)\n"
    "  lo / hi      подвёл до КАСАНИЯ — записать предел (отступ внутрь скетч сделает сам)\n"
    "  mg <град>    величина отступа от касания (3)\n"
    "  rst lo|hi|z|all  снять предел/нуль ТЕКУЩЕГО канала (чтобы перемерить)\n"
    "  lim          показать нули и пределы всех каналов\n"
    "  fac          вернуть таблицу из spotty_config.h (источник правды)\n"
    "  hist         сверка исторического замера 20.08 с конфигом\n"
    "  exp          выдать готовые массивы для прошивки\n"
    "  nvs / ld / er  записать во флеш / прочитать / стереть (только свой ключ)\n"
    "  sv           сохранить результат канала\n"
    "  tab          таблица для CALIBRATION.md\n"
    "\n  ПАРАМЕТРЫ\n"
    "  win <lo> <hi> рабочее окно, мкс (кламп)\n"
    "  ss <us>      размер шага (10)\n"
    "  ph <ms>      длительность клевка (500)\n"
    "  sl <us/s>    скорость перехода (400)\n"
    "  hz <f>       частота ШИМ (330; историческое 50 грубее мёртвой зоны)\n"
    "  vb <мВ>      измеренное напряжение питания серво\n"
    "  dt <текст>   дата для таблицы\n"
    "  who <текст>  кто измерял\n"
    "  i            скан I2C     st  состояние     h  справка\n"
  ));
  spottyOscHelp(Serial);
}

static void i2cScan() {
  Serial.println(F("I2C scan (SDA=8, SCL=9):"));
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X%s\n", a, (a == PCA_ADDR) ? "   <- PCA9685" : "");
      found++;
    }
  }
  if (!found) Serial.println(F("  НИЧЕГО НЕ НАЙДЕНО."));
}

static void handleLine(char *s) {
  while (*s == ' ') s++;
  if (!*s) return;

  char cmd[8] = {0};
  int n = 0;
  while (s[n] && s[n] != ' ' && n < 7) { cmd[n] = s[n]; n++; }
  cmd[n] = 0;
  const char *rest = s + n;
  while (*rest == ' ') rest++;
  bool hasArg = (*rest != 0);
  long arg = hasArg ? atol(rest) : 0;

  if (!strcmp(cmd, "x"))   { allOff(); Serial.println(F("ВСЁ ОБЕСТОЧЕНО.")); return; }
  if (!strcmp(cmd, "h"))   { help(); return; }
  if (!strcmp(cmd, "osc")) {                 // генератор PCA9685, spotty_osc.h
    if (spottyOscCommand(rest, pca, powered, Serial) && powered && curCh >= 0)
      spottyWriteUs(pca, (uint8_t)curCh, curUs);   // тот же импульс с новой K
    return;
  }
  if (!strcmp(cmd, "st"))  { status(); return; }
  if (!strcmp(cmd, "i"))   { i2cScan(); return; }
  if (!strcmp(cmd, "rel")) { releaseCh(); return; }

  if (!strcmp(cmd, "ch")) {
    if (!hasArg || arg < 0 || arg > 15) { Serial.println(F("ch <0..15>")); return; }
    allOff();
    curCh = (int8_t)arg;
    nPts  = 0;
    curUs = SPEC_NEUTRAL_US;
    Serial.printf("канал %ld выбран, все каналы обесточены, точки сброшены\n", arg);
    Serial.println(F("Дальше: 'g 1520' — встать в паспортную нейтраль."));
    return;
  }

  if (!strcmp(cmd, "g")) {
    if (!hasArg) { Serial.println(F("g <us>")); return; }
    uint16_t t = clampWin(arg);
    if ((long)t != arg) Serial.printf("[!] %ld вне окна, ограничено до %u\n", arg, t);
    slewTo(t);
    Serial.printf("импульс %u мкс, удержание\n", curUs);
    return;
  }
  if (!strcmp(cmd, "+") || !strcmp(cmd, "-")) {
    int32_t t = (int32_t)curUs + (cmd[0] == '+' ? stepUs : -(int32_t)stepUs);
    uint16_t c2 = clampWin(t);
    if (c2 != t) Serial.println(F("[!] упёрлись в край окна ('win' чтобы расширить)"));
    slewTo(c2);
    Serial.printf("импульс %u мкс\n", curUs);
    return;
  }
  if (!strcmp(cmd, "pr")) {
    if (!hasArg) { Serial.println(F("pr <us>")); return; }
    uint16_t t = clampWin(arg);
    if ((long)t != arg) Serial.printf("[!] %ld вне окна, ограничено до %u\n", arg, t);
    slewTo(t);
    Serial.printf("клевок: %u мкс, удержание %u мс\n", curUs, probeMs);
    delay(probeMs);
    releaseCh();
    Serial.println(F("Смотри транспортир СЕЙЧАС: положение держится трением\n"
                     "редуктора. Если качалка уползла — снимай угол под 'g'."));
    return;
  }

  if (!strcmp(cmd, "p")) {
    if (!hasArg) { Serial.println(F("p <угол в градусах>")); return; }
    if (curCh < 0) { Serial.println(F("сначала 'ch <n>'")); return; }
    if (!powered)  { Serial.println(F("[!] ШИМ снят. Точка привязывается к УДЕРЖИВАЕМОМУ\n"
                                      "    импульсу — сначала 'g <us>'.")); return; }
    if (nPts >= MAX_PTS) { Serial.println(F("буфер точек полон")); return; }
    ptUs[nPts]  = curUs;
    ptDeg[nPts] = atof(rest);
    nPts++;
    Serial.printf("точка %u: %u мкс = %.1f град\n", nPts, curUs, ptDeg[nPts - 1]);
    if (nPts >= 2) doFit(false);
    return;
  }
  if (!strcmp(cmd, "pts")) { printPts(); return; }
  if (!strcmp(cmd, "dp")) {
    if (nPts) { nPts--; Serial.printf("удалена, осталось %u\n", nPts); }
    else      { Serial.println(F("точек нет")); }
    return;
  }
  if (!strcmp(cmd, "clr")) { nPts = 0; Serial.println(F("точки стёрты")); return; }
  if (!strcmp(cmd, "fit")) { doFit(false); return; }

  // lo/hi снимаются В ТОЧКЕ КАСАНИЯ. Отступ внутрь скетч делает сам:
  // записывать сам упор нельзя, при нагреве и просадке питания серво
  // дойдёт до него и встанет в натяг.
  if (!strcmp(cmd, "lo") || !strcmp(cmd, "hi")) {
    if (curCh < 0) { Serial.println(F("сначала 'ch <n>'")); return; }
    uint16_t marg = (uint16_t)(marginDeg * 5.57f);
    bool low = (cmd[0] == 'l');
    uint16_t lim = low ? (uint16_t)(curUs + marg) : (uint16_t)(curUs - marg);
    if (low) cal[curCh].lo = lim; else cal[curCh].hi = lim;
    Serial.printf("канал %d: касание на %u мкс, %s предел записан как %u мкс (отступ %.1f град)\n",
                  curCh, curUs, low ? "нижний" : "верхний", lim, marginDeg);
    Serial.println(F("Дальше 'g' за этот предел не пойдёт. 'nvs' — записать во флеш."));
    return;
  }
  if (!strcmp(cmd, "z")) {
    if (curCh < 0) { Serial.println(F("сначала 'ch <n>'")); return; }
    cal[curCh].zero = curUs;
    Serial.printf("канал %d: нуль сустава = %u мкс (отклонение от 1480: %+d мкс = %+.1f град)\n",
                  curCh, curUs, (int)curUs - 1480, ((int)curUs - 1480) / 5.57f);
    return;
  }
  if (!strcmp(cmd, "mg") && hasArg) {
    if (arg < 0) arg = 0;
    if (arg > 20) arg = 20;
    marginDeg = (float)arg;
    Serial.printf("отступ от касания = %.1f град = %.0f мкс\n", marginDeg, marginDeg * 5.57f);
    return;
  }
  // Точечный сброс. Нужен потому, что записанный предел СРАЗУ ограничивает
  // 'g': не сбросив его, невозможно подъехать к точке касания и перемерить.
  if (!strcmp(cmd, "rst")) {
    if (curCh < 0) { Serial.println(F("сначала 'ch <n>'")); return; }
    Cal &e = cal[curCh];
    if (!strcmp(rest, "lo"))       { e.lo = 0;  Serial.printf("канал %d: нижний предел снят\n", curCh); }
    else if (!strcmp(rest, "hi"))  { e.hi = 0;  Serial.printf("канал %d: верхний предел снят\n", curCh); }
    else if (!strcmp(rest, "z"))   { e.zero = 0; Serial.printf("канал %d: нуль снят\n", curCh); }
    else if (!strcmp(rest, "all")) { e.lo = e.hi = e.zero = 0;
                                     Serial.printf("канал %d: нуль и оба предела сняты\n", curCh); }
    else { Serial.println(F("rst lo | rst hi | rst z | rst all   (только текущий канал)")); return; }
    Serial.printf("теперь ход ограничен только окном %u..%u мкс\n", winLo, winHi);
    Serial.println(F("Во флеше пока по-старому: 'nvs' чтобы записать, 'ld' чтобы откатить."));
    return;
  }
  if (!strcmp(cmd, "fac")) {
    Serial.println(F("возврат к таблице из spotty_config.h:"));
    applyConfig(true);
    return;
  }
  if (!strcmp(cmd, "hist")) { histCompare(); return; }
  if (!strcmp(cmd, "lim")) { printLimits(); return; }
  if (!strcmp(cmd, "exp")) { exportArrays(); return; }
  if (!strcmp(cmd, "nvs")) { nvsSave(); return; }
  if (!strcmp(cmd, "ld"))  { nvsLoad(false); return; }
  if (!strcmp(cmd, "er")) {
    // РЕВИЗИЯ 2026-09-12. Было prefs.clear() — оно стирало ВСЁ пространство
    // "spotty", включая чужой ключ "hold" от servo_hold. Сосед терял
    // подстроенную позу сборки без единого слова в консоли. Удаляем только
    // свой ключ.
    prefs.begin("spotty", false);
    bool had = prefs.isKey("lim");
    prefs.remove("lim");
    prefs.end();
    applyConfig(false);          // не «сброшены в ноль», а назад к конфигу
    Serial.println(had ? F("флеш очищен: нули и пределы снова из spotty_config.h")
                       : F("во флеше и так ничего не было"));
    Serial.println(F("ключ 'hold' от servo_hold не тронут"));
    return;
  }
  if (!strcmp(cmd, "sv"))  { saveCh(); return; }
  if (!strcmp(cmd, "tab")) { printTable(); return; }

  if (!strcmp(cmd, "win")) {
    long a = 0, b = 0;
    if (sscanf(rest, "%ld %ld", &a, &b) != 2) { Serial.println(F("win <lo> <hi>")); return; }
    if (a < ABS_MIN_US || b > ABS_MAX_US || a >= b) {
      Serial.printf("окно обязано лежать в %u..%u и lo<hi\n", ABS_MIN_US, ABS_MAX_US); return;
    }
    winLo = (uint16_t)a; winHi = (uint16_t)b;
    Serial.printf("окно %u..%u мкс\n", winLo, winHi);
    Serial.println(F("[!] расширил окно — края ищи ТОЛЬКО 'pr', не 'g'."));
    return;
  }
  if (!strcmp(cmd, "ss") && hasArg) { if (arg < 1) arg = 1; if (arg > 200) arg = 200;
    stepUs = (uint16_t)arg; Serial.printf("шаг = %u мкс\n", stepUs); return; }
  if (!strcmp(cmd, "ph") && hasArg) { if (arg < 100) arg = 100; if (arg > 3000) arg = 3000;
    probeMs = (uint16_t)arg; Serial.printf("клевок = %u мс\n", probeMs); return; }
  if (!strcmp(cmd, "sl") && hasArg) { if (arg < 50) arg = 50; if (arg > 5000) arg = 5000;
    slewUsS = (uint16_t)arg; Serial.printf("скорость = %u мкс/с\n", slewUsS); return; }
  if (!strcmp(cmd, "hz") && hasArg) {
    if (arg < 40 || arg > 400) { Serial.println(F("частота 40..400 Гц")); return; }
    allOff();
    pwmHz = (float)arg;
    pca.setPWMFreq(pwmHz);
    delay(10);
    Serial.printf("частота %.0f Гц, шаг PCA9685 = %.2f мкс\n",
                  pwmHz, (1000000.0f / pwmHz) / 4096.0f);
    Serial.println(F("Все каналы обесточены, частота меняет ширину импульса —\n"
                     "ранее снятые точки к новой частоте не относятся."));
    return;
  }
  if (!strcmp(cmd, "vb") && hasArg) {
    vBatMv = (uint32_t)arg;
    Serial.printf("питание серво = %.2f В\n", vBatMv / 1000.0f);
    if (vBatMv > 7400) Serial.println(F("[!!] ВЫШЕ ПАСПОРТНЫХ 7.4 В. Серво вне режима."));
    else if (vBatMv < 5000) Serial.println(F("[!] НИЖЕ паспортных 5.0 В."));
    return;
  }
  if (!strcmp(cmd, "dt"))  { strncpy(dateStr, rest, sizeof(dateStr) - 1); Serial.printf("дата: %s\n", dateStr); return; }
  if (!strcmp(cmd, "who")) { strncpy(whoStr, rest, sizeof(whoStr) - 1);  Serial.printf("кто: %s\n", whoStr);  return; }

  Serial.println(F("неизвестная команда, 'h' — справка"));
}

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    rxTotal++; lastRxMs = millis();
    if (echoOn) { if (c == '\r' || c == '\n') Serial.println(); else Serial.write(c); }
    if (c == '\r' || c == '\n') {
      if (lineLen) { line[lineLen] = 0; handleLine(line); lineLen = 0; }
      continue;
    }
    if (lineLen < sizeof(line) - 1) line[lineLen++] = c;
  }
  if (lineLen && (millis() - lastRxMs) >= LINE_IDLE_MS) {
    line[lineLen] = 0;
    if (echoOn) Serial.println();
    handleLine(line);
    lineLen = 0;
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_HZ);

  Serial.println(F("\n===================================================="));
  Serial.println(F(" Spotty — servo_calib: импульс <-> угол, транспортир"));
  Serial.println(F("===================================================="));
  i2cScan();

  pca.begin();
  pca.setOscillatorFrequency(SPOTTY_OSC_HZ);   // 27 МГц, как при калибровке
  pca.setPWMFreq(pwmHz);
  delay(10);
  allOff();
  spottyOscBoot(pca, Serial);   // генератор PCA9685: до первой подачи ШИМ

  // Порядок важен: сперва конфиг как несгораемый минимум, затем поверх —
  // NVS, если она есть. Сверка с историческим замером печатается между ними,
  // чтобы расхождение нулей нельзя было пропустить.
  Serial.println(F("\nРабочая таблица (spotty_config.h — источник правды):"));
  applyConfig(true);
  histCompare();
  nvsLoad(false);

  Serial.println(F("\nВсе каналы обесточены. Движения при старте НЕТ."));
  Serial.println(F("ПРИ СБРОСЕ MCU серво НЕ проворачивается: PCA9685 держит"));
  Serial.println(F("последний импульс сам. Но при инициализации выходы гаснут,"));
  Serial.println(F("и серво становится мягким — нога упадёт. Подпирай при сборке."));
  Serial.printf("Частота %.0f Гц, шаг PCA9685 %.2f мкс, мёртвая зона серво 3 мкс\n",
                pwmHz, (1000000.0f / pwmHz) / 4096.0f);
  Serial.printf("ИЗМЕРЕНО: ход %u..%u мкс = 0..355 град, 5.54 мкс/град\n",
                ABS_MIN_US, ABS_MAX_US);
  Serial.println(F("Паспорт (листинги, опровергнут в части хода): 5.0..7.4 В, мёртвая зона 3 мкс"));
  Serial.println(F("Выше 2400 мкс линейность хуже (6.25 мкс/град) — точки туда не ставить."));
  Serial.println(F("\nПЕРЕД ПЕРВЫМ ДВИЖЕНИЕМ:"));
  Serial.println(F("  1. измерь напряжение на V+ мультиметром и введи 'vb <мВ>';"));
  Serial.println(F("  2. заряженная 2S даёт 8.4 В — это ВЫШЕ паспортных 7.4 В;"));
  Serial.println(F("  3. разрыв питания серво держи под рукой: это единственный"));
  Serial.println(F("     аварийный останов, стойло здесь не детектируется."));
  help();
  status();
}

void loop() {
  pollSerial();
  uint32_t now = millis();
  // Генератор PCA9685: фоновое слежение, раз в 2 с. Сменилась K — тот же импульс заново.
  if (spottyOscTrackTick(pca, Serial) && powered && curCh >= 0)
    spottyWriteUs(pca, (uint8_t)curCh, curUs);
  if ((now - lastBeat) >= BEAT_MS) {
    lastBeat = now;
    if (curCh < 0) Serial.printf("[ожидание]  канал не выбран  принято байт: %lu\n",
                                 (unsigned long)rxTotal);
  }
}
