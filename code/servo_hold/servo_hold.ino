/*
 * servo_hold.ino — удержание всех суставов в известной позе на время сборки
 * Проект Spotty (OpenQuadruped).
 * ---------------------------------------------------------------------------
 * ЗАЧЕМ
 *   Обесточенное серво мягкое: деталь уползает под своим весом, от толчка,
 *   от натяжения провода. На собираемой раме это ломает детали. Скетч
 *   непрерывно удерживает каждый канал в позе, в которой он был собран.
 *
 * ЧТО КУДА
 *   ВСЕ двенадцать каналов — на свои нули из spotty_config.h (ревизия
 *   2026-08-28). Прежняя редакция этой строки обещала 1500 мкс на всё, кроме
 *   коленей; это давно неправда, цели берутся из JOINT_ZERO_US и у каждого
 *   канала свои. Локальная TABLE[] — только названия для печати и сверка.
 *
 * ГЛАВНАЯ ОПАСНОСТЬ И КАК ОНА ЗАКРЫТА
 *   Подача ШИМ на закреплённое серво, стоящее НЕ там, куда его зовут, — это
 *   рывок на полном моменте. Именно это ломает корпуса. Здесь:
 *     - цели равны позам сборки, то есть тому, где детали уже стоят;
 *     - при старте даётся обратный отсчёт, любая клавиша его отменяет;
 *     - 'x' снимает ШИМ мгновенно, в том числе во время удержания.
 *
 * СТОРОЖ ДРАЙВЕРА
 *   Если PCA9685 просядет по питанию и сбросится, его выходы погаснут и всё
 *   станет мягким — снаружи это выглядит как «серво само поехало». Скетч раз
 *   в 500 мс читает PRE_SCALE и MODE1. Обнаружив сброс, он переинициализирует
 *   драйвер и ВОССТАНАВЛИВАЕТ удержание, а не ждёт оператора.
 *
 * ЧЕГО СКЕТЧ НЕ ДЕЛАЕТ
 *   Не знает фактического положения вала: обратной связи в железе нет.
 *   «Удерживает» означает «выдаёт импульс», а не «проверил и подтвердил».
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
#if defined(ARDUINO_ARCH_ESP32)
  #include <esp_system.h>
#endif

static const uint8_t  I2C_SDA_PIN = 8;
static const uint8_t  I2C_SCL_PIN = 9;
static const uint8_t  PCA_ADDR    = 0x40;
static const uint32_t I2C_HZ      = 400000UL;
static const float    PWM_HZ      = 330.0f;

static const uint8_t PCAREG_MODE1    = 0x00;
static const uint8_t PCAREG_PRESCALE = 0xFE;
static const uint8_t PCABIT_SLEEP    = 0x10;

// Жёсткие границы: измерено 2026-08-20, линейный участок серво.
// РЕВИЗИЯ 2026-08-28: собственных копий этих чисел здесь больше нет,
// они берутся из spotty_config.h. Разъезд двух копий физически невозможен.
static const uint16_t ABS_MIN_US = SPOTTY_ABS_MIN_US;
static const uint16_t ABS_MAX_US = SPOTTY_ABS_MAX_US;

// ---------------------------------------------------------------------------
// КАРТА СУСТАВОВ И ПОЗ. Разводка сообщена оператором 2026-08-20.
//
// Схема разводки: каналы сгруппированы ПО ТИПУ СУСТАВА, не по ноге.
//   колени  0, 1, 13, 14
//   бёдра   2, 4, 10, 12
//   хипы    5, 6, 8, 9
// Диагонально противоположные ноги дают в сумме 14 на каждом типе сустава:
//   BL0+FR14, BL2+FR12, BL5+FR9, BR1+FL13, BR4+FL10, BR6+FL8.
// Это годится как проверка на перепутанный провод.
//
// ПРЕДЕЛЫ.
//   ВСЕ ДВЕНАДЦАТЬ суставов измерены физически 2026-08-20: подвод малым шагом
//   до касания, затем отступ 3° внутрь. Временных значений больше нет.
// ---------------------------------------------------------------------------
// ПЕРЕИМЕНОВАН 2026-08-28: в spotty_config.h есть свой enum Joint
// (JOINT_HIP/JOINT_THIGH/JOINT_KNEE — порядок суставов внутри ноги).
// Два разных enum Joint в одной единице трансляции не собираются.
// Здешний перечень — про НАЗВАНИЕ сустава в печати, назначение другое.
enum HoldJoint : uint8_t { J_NONE, J_KNEE, J_THIGH, J_HIP };
struct Hold {
  uint8_t  ch;
  uint16_t us, lo, hi;
  bool     used, measured;
  const char *leg;
  HoldJoint joint;
};

// Все пределы ИЗМЕРЕНЫ физически 2026-08-20 (подвод до касания, отступ 3°).
//
// РЕВИЗИЯ 2026-08-28. Поля .us, .lo, .hi больше НЕ УЧАСТВУЮТ в работе: цели
// берутся из JOINT_ZERO_US, кламп — из spottyClampUs, оба из spotty_config.h.
// Таблица оставлена как источник названий ноги и сустава для печати, а её
// числа синхронизированы с конфигом, чтобы сверка при старте молчала.
// Расхождение = ошибка синхронизации, и увидеть его надо сразу, а не искать
// в шуме из накопившихся правок. История замеров живёт в CALIBRATION.md.
// Синхронизировано с ревизией нулей 2026-08-28: ch5, ch9, ch13, ch14.
// Синхронизировано с ревизией нулей 2026-08-31: ch4 1460->1450,
// ch10 1470->1450, ch12 1540->1530. До этого сверка при старте печатала три
// расхождения каждый раз, и в этом шуме утонуло бы четвёртое, настоящее.
static const Hold TABLE[16] = {
  {  0, 1520,  700, 1780, true , true , "BL задняя левая", J_KNEE  },
  {  1, 1440, 1200, 2250, true , true , "BR задняя правая", J_KNEE  },
  {  2, 1460, 1080, 1980, true , true , "BL задняя левая", J_THIGH },
  {  3,    0,    0,    0, false, false, "-", J_NONE  },
  {  4, 1450,  960, 1820, true , true , "BR задняя правая", J_THIGH },
  {  5, 1470, 1350, 1780, true , true , "BL задняя левая", J_HIP   },
  {  6, 1430, 1130, 1550, true , true , "BR задняя правая", J_HIP   },
  {  7,    0,    0,    0, false, false, "-", J_NONE  },
  {  8, 1430, 1140, 1560, true , true , "FL передняя левая", J_HIP   },
  {  9, 1420, 1350, 1780, true , true , "FR передняя правая", J_HIP   },
  { 10, 1450, 1010, 1990, true , true , "FL передняя левая", J_THIGH },
  { 11,    0,    0,    0, false, false, "-", J_NONE  },
  { 12, 1530, 1000, 1890, true , true , "FR передняя правая", J_THIGH },
  { 13, 1500,  710, 1730, true , true , "FL передняя левая", J_KNEE  },
  { 14, 1460, 1220, 2290, true , true , "FR передняя правая", J_KNEE  },
  { 15,    0,    0,    0, false, false, "-", J_NONE  },
};

static uint16_t clampCh(uint8_t ch, int32_t us);

static const char *jointName(HoldJoint j) {
  return j == J_KNEE ? "колено" : j == J_THIGH ? "бедро" : j == J_HIP ? "хип" : "-";
}

// Сколько секунд на отмену автозахвата при старте.
static const uint8_t ARM_SECONDS = 3;

Adafruit_PWMServoDriver pca(PCA_ADDR);

static uint16_t targetUs[16];   // текущая цель удержания
static uint16_t origUs[16];     // цель из таблицы в исходнике, для 'r' и дельты
static bool     held[16];
static int8_t   selCh   = -1;   // канал, выбранный для подстройки
static uint16_t stepUs  = 5;    // шаг подстройки, мкс (5 мкс = 0.9 град)
static Preferences prefs;

// Критерий «поправка велика».
// ПЕРВАЯ РЕДАКЦИЯ СРАВНИВАЛА С ПОЛОВИНОЙ ЗУБА (40 мкс) И БЫЛА НЕВЕРНА:
// половина зуба ничего не говорит о том, вредна поправка или нет. Значение
// имеет только одно — насколько нуль сустава отстоит от ЦЕНТРА рабочего
// окна, потому что именно это создаёт перекос доступного хода.
// Центр окна 520..2380 равен 1450 мкс, а не 1500.
static const uint16_t WINDOW_CENTER_US = (ABS_MIN_US + ABS_MAX_US) / 2;
static const uint16_t OFFCENTER_WARN_US = 150;   // ~27 град перекоса
static bool     engaged   = false;
static uint32_t lastCheck = 0;
static uint32_t lastBeat  = 0;
static uint8_t  refPrescale = 0;
static bool     refValid    = false;
static uint32_t recoveries  = 0;
static uint32_t i2cErrRun   = 0;
static const uint32_t CHECK_MS = 500;
static const uint32_t BEAT_MS  = 10000;

static char     line[48];
static uint8_t  lineLen = 0;
static uint32_t lastRxMs = 0;
static const uint32_t LINE_IDLE_MS = 120;

// ------------------------------- утилиты -----------------------------------
static const Hold &entry(uint8_t ch) { return TABLE[ch]; }

// Насколько нуль канала смещён от центра окна и во что это обходится.
static void offcenterReport(uint8_t ch, uint16_t us) {
  int off = (int)us - (int)WINDOW_CENTER_US;
  float dn = (us - ABS_MIN_US) / 5.57f, up = (ABS_MAX_US - us) / 5.57f;
  Serial.printf("   от центра окна %+d мкс, ход вниз %.0f° вверх %.0f°, перекос %.0f°%s\n",
                off, dn, up, dn > up ? dn - up : up - dn,
                (abs(off) > OFFCENTER_WARN_US) ? "   <- заметный перекос" : "");
}

static void buildTargets() {
  for (uint8_t ch = 0; ch < 16; ch++) {
    // РЕВИЗИЯ 2026-08-28: нуль берётся из spotty_config.h, а не из TABLE[].us.
    // Копия нуля в двух файлах уже разъехалась однажды (ch4: 1460 против
    // измеренного 1445) и разъедется снова. Теперь копии нет.
    bool use     = spottyChUsed(ch);
    targetUs[ch] = use ? clampCh(ch, JOINT_ZERO_US[ch]) : 0;
    held[ch]     = use;
    origUs[ch]   = targetUs[ch];
  }
}

/*
 * ОЧИСТКА ФЛЕША. Нужна чаще, чем кажется.
 *
 * nvsLoad() при старте перекрывает нули из конфига значениями из флеша — так
 * задумано, это подстройка позы во время сборки. Но у этого есть оборотная
 * сторона: ОБНОВИЛ НУЛИ В КОНФИГЕ, ПЕРЕЗАЛИЛ, А РОБОТ СТОИТ ПО-СТАРОМУ.
 * Флеш перезаливку переживает, и старая копия продолжает побеждать.
 *
 * До этой ревизии стереть её было нечем. Теперь есть.
 */
static void applyAll();          // определены ниже, нужны здесь
static void buildTargets();

static void nvsClear() {
  prefs.begin("spotty", false);
  bool had = prefs.isKey("hold");
  prefs.remove("hold");
  prefs.end();
  Serial.println(had ? F("флеш очищен: цели снова берутся из spotty_config.h")
                     : F("во флеше и так ничего не было"));
  buildTargets();
  applyAll();
  Serial.println(F("цели перечитаны из конфига и применены"));
}

static void nvsSave() {
  prefs.begin("spotty", false);
  prefs.putBytes("hold", targetUs, sizeof(targetUs));
  prefs.end();
  Serial.println(F("цели удержания записаны во флеш"));
}

static void nvsLoad(bool quiet) {
  uint16_t t[16];
  prefs.begin("spotty", true);
  size_t n = prefs.getBytes("hold", t, sizeof(t));
  prefs.end();
  if (n != sizeof(t)) { if (!quiet) Serial.println(F("во флеше целей нет")); return; }
  uint8_t cnt = 0, over = 0;
  for (uint8_t ch = 0; ch < 16; ch++) {
    if (!spottyChUsed(ch) || !t[ch]) continue;
    uint16_t v = clampCh(ch, t[ch]);     // флеш тоже проходит жёсткий кламп
    // Флеш ГЛАВНЕЕ конфига по прямому назначению скетча (подстройка позы во
    // время сборки), но оператору обязаны сказать, что нуль из конфига
    // перекрыт, и на сколько градусов.
    if (v != JOINT_ZERO_US[ch]) {
      int d = (int)v - (int)JOINT_ZERO_US[ch];
      Serial.printf("  ch%-2u флеш %u перекрывает нуль конфига %u (%+d мкс = %+.1f град)\n",
                    ch, v, JOINT_ZERO_US[ch], d, d / SPOTTY_US_PER_DEG);
      over++;
    }
    targetUs[ch] = v; cnt++;
  }
  Serial.printf("из флеша прочитано целей: %u, из них перекрывают конфиг: %u\n", cnt, over);
  if (over) Serial.println(F("  ('ld' перечитает флеш, 'r' по каналу вернёт исходное)"));
}

// Печатает всё, что отклонилось от таблицы в исходнике, готовыми строками.
static void exportDeltas() {
  Serial.println(F("\n--- отклонения от таблицы в исходнике ---"));
  uint8_t cnt = 0;
  for (uint8_t ch = 0; ch < 16; ch++) {
    int d = (int)targetUs[ch] - (int)origUs[ch];
    if (!d) continue;
    Serial.printf("  ch%-3u %u -> %u   (%+d мкс = %+.1f град)%s\n",
                  ch, origUs[ch], targetUs[ch], d, d / 5.57f,
                  "");
    offcenterReport(ch, targetUs[ch]);
    cnt++;
  }
  if (!cnt) { Serial.println(F("  (ничего не менялось)")); return; }
  Serial.println(F("\nГотовая таблица TABLE для servo_hold.ino:"));
  for (uint8_t ch = 0; ch < 16; ch++)
    if (TABLE[ch].used)
      Serial.printf("  ch%-3u %-20s %-7s нуль %u\n", ch, TABLE[ch].leg,
                    jointName(TABLE[ch].joint), targetUs[ch]);
  Serial.println(F("Флеш переживёт сброс, но не перезаливку. Перенеси в исходник."));
}

// Кламп по пределам КОНКРЕТНОГО канала. Это и есть защита рамы: за границы
// сустава команда не пройдёт ни через '+', ни через 'p', ни через флеш.
//
// РЕВИЗИЯ 2026-08-28. Раньше кламп считался здесь по локальной копии таблицы.
// Теперь он делегирован общему модулю spotty_config.h и потому:
//   - совпадает с клампом gait_test до микросекунды;
//   - включает запас SPOTTY_LIMIT_MARGIN_DEG внутрь от измеренного предела;
//   - проверен на этапе компиляции (static_assert по каждому каналу).
// Локальные TABLE[].lo/.hi остаются только как исторический протокол замера
// и на кламп больше не влияют.
static uint16_t clampCh(uint8_t ch, int32_t us) {
  if (ch > 15) return ABS_MIN_US;
  if (!spottyChUsed(ch)) {                       // канала нет в конфиге
    int32_t v = us < ABS_MIN_US ? ABS_MIN_US : (us > ABS_MAX_US ? ABS_MAX_US : us);
    return (uint16_t)v;
  }
  return spottyClampUs(ch, (long)us);
}

// Отчёт о расхождении локальной таблицы с конфигом. Печатается один раз при
// старте. Расхождение — это ОШИБКА (см. шапку spotty_config.h): правится в
// конфиге, оттуда переносится сюда. Скетч работает по конфигу в любом случае.
static uint8_t configDiffReport() {
  uint8_t diff = 0;
  Serial.println(F("\n--- сверка локальной таблицы с spotty_config.h ---"));
  for (uint8_t ch = 0; ch < 16; ch++) {
    bool cfg = spottyChUsed(ch);
    if (!cfg && !TABLE[ch].used) continue;
    if (cfg != TABLE[ch].used) {
      Serial.printf("ch%-2u РАСХОЖДЕНИЕ: конфиг %s, локально %s\n",
                    ch, cfg ? "используется" : "свободен",
                    TABLE[ch].used ? "используется" : "свободен");
      diff++; continue;
    }
    if (TABLE[ch].us != JOINT_ZERO_US[ch])
      { Serial.printf("ch%-2u нуль: локально %u, конфиг %u  <- берётся КОНФИГ\n",
                      ch, TABLE[ch].us, JOINT_ZERO_US[ch]); diff++; }
    if (TABLE[ch].lo != JOINT_MIN_US[ch] || TABLE[ch].hi != JOINT_MAX_US[ch])
      { Serial.printf("ch%-2u предел: локально %u..%u, конфиг %u..%u  <- берётся КОНФИГ\n",
                      ch, TABLE[ch].lo, TABLE[ch].hi, JOINT_MIN_US[ch], JOINT_MAX_US[ch]); diff++; }
  }
  if (diff) Serial.printf("Расхождений: %u. Это ОШИБКА синхронизации, править в spotty_config.h.\n", diff);
  else      Serial.println(F("Расхождений нет."));
  return diff;
}

static void chOff(uint8_t ch) { pca.setPWM(ch, 0, 4096); }

static void applyAll() {
  for (uint8_t ch = 0; ch < 16; ch++) {
    if (engaged && held[ch] && spottyChUsed(ch)) spottyWriteUs(pca, ch, clampCh(ch, targetUs[ch]));
    else if (ch != SPOTTY_OSC_PROBE_CH) chOff(ch);   // канал зонда генератора не трогаем: на нём меандр фонового слежения (spotty_osc.h)
  }
}

static void releaseAll() {
  engaged = false;
  for (uint8_t ch = 0; ch < 16; ch++) if (ch != SPOTTY_OSC_PROBE_CH) chOff(ch);   // канал зонда генератора не трогаем: на нём меандр фонового слежения (spotty_osc.h)
  Serial.println(F("ШИМ снят со всех каналов. Всё мягкое — подпирай детали."));
}

static void engageAll() {
  engaged = true;
  applyAll();
  Serial.println(F("УДЕРЖАНИЕ ВКЛЮЧЕНО."));
}

// --------------------------- низкий уровень I2C ----------------------------
static bool pcaRead(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)PCA_ADDR, 1) != 1) return false;
  val = (uint8_t)Wire.read();
  return true;
}

static void initDriver() {
  pca.begin();
  pca.setOscillatorFrequency(SPOTTY_OSC_HZ);   // 27 МГц, как при калибровке
  pca.setPWMFreq(PWM_HZ);
  delay(10);
  if (pcaRead(PCAREG_PRESCALE, refPrescale)) refValid = true;
}

// Возвращает true, если пришлось восстанавливать.
static bool watchdog() {
  uint8_t ps = 0, m1 = 0;
  bool ok = pcaRead(PCAREG_PRESCALE, ps) && pcaRead(PCAREG_MODE1, m1);

  if (!ok) {
    i2cErrRun++;
    if (i2cErrRun >= 4) {
      Serial.printf("[!] I2C не отвечает (подряд %lu). Переинициализация.\n",
                    (unsigned long)i2cErrRun);
      i2cErrRun = 0;
      initDriver();
      applyAll();
      recoveries++;
      return true;
    }
    return false;
  }
  i2cErrRun = 0;

  bool broken = (refValid && ps != refPrescale) || (m1 & PCABIT_SLEEP);
  if (!broken) return false;

  recoveries++;
  Serial.printf("[!!] PCA9685 СБРОСИЛСЯ (PRE_SCALE %u->%u, MODE1 0x%02X).\n",
                refPrescale, ps, m1);
  Serial.println(F("     Выходы погасли, суставы стали мягкими. Восстанавливаю."));
  initDriver();
  applyAll();
  Serial.printf("     Удержание восстановлено. Всего восстановлений: %lu\n",
                (unsigned long)recoveries);
  Serial.println(F("     ПРИЧИНА — просадка питания логики. Это надо чинить,"));
  Serial.println(F("     сторож лишь смягчает последствия."));
  return true;
}

// ------------------------------- вывод -------------------------------------
static void status() {
  Serial.println(F("\n--- удержание ---"));
  Serial.printf("состояние     : %s\n", engaged ? "ВКЛЮЧЕНО" : "снято (всё мягкое)");
  Serial.printf("восстановлений: %lu\n", (unsigned long)recoveries);
  spottyOscStatus(Serial);
  Serial.printf("жёсткий предел: измеренное окно минус %.1f° с каждой стороны\n",
                SPOTTY_LIMIT_MARGIN_DEG);
  Serial.println(F("  ch   мкс    нога                 сустав  жёсткий  запас вниз/вверх, °  статус"));
  for (uint8_t ch = 0; ch < 16; ch++) {
    if (!spottyChUsed(ch)) { Serial.printf("  %-4u   ---   не используется\n", ch); continue; }
    float alo, ahi; spottyAngleLimits(ch, alo, ahi);
    float cur = ((float)targetUs[ch] - (float)JOINT_ZERO_US[ch])
                / (JOINT_SIGN[ch] * SPOTTY_US_PER_DEG);
    Serial.printf("  %-4u %-6u %-20s %-7s %4u..%-4u  %6.1f/%-6.1f  %s\n",
                  ch, targetUs[ch], TABLE[ch].leg, jointName(TABLE[ch].joint),
                  spottyHardLoUs(ch), spottyHardHiUs(ch),
                  cur - alo, ahi - cur,
                  held[ch] ? "" : "<- ОТПУЩЕН");
  }
}

static void help() {
  Serial.println(F(
    "\n=== servo_hold — команды ===\n"
    "  x            СНЯТЬ ШИМ со всех каналов (всё станет мягким)\n"
    "  e            включить удержание\n"
    "  o <ch>       отпустить ОДИН канал (чтобы работать с ним руками)\n"
    "  n <ch>       вернуть удержание одному каналу\n"
    "  p <ch> <us>  сдвинуть цель одного канала и удерживать там\n"
    "  s            состояние\n"
    "  ?            справка\n"
    "  --- пошаговая подстройка нуля ---\n"
    "  c <ch>       выбрать канал для подстройки\n"
    "  + / -        сдвинуть выбранный канал на шаг\n"
    "  ss <us>      размер шага (5 мкс = 0.9 град)\n"
    "  r            вернуть выбранный канал к значению из таблицы\n"
    "  w            показать все отклонения и строки для исходника\n"
    "  nvs / ld     записать цели во флеш / прочитать\n"
    "  clr          СТЕРЕТЬ флеш: вернуться к нулям из spotty_config.h\n"
    "\n  Отпущенный канал мягкий: деталь надо подпирать."
  ));
  spottyOscHelp(Serial);
}

// ------------------------------ команды ------------------------------------
static void handleLine(char *s) {
  while (*s == ' ') s++;
  if (!*s) return;

  char cmd[6] = {0};
  int n = 0;
  while (s[n] && s[n] != ' ' && n < 5) { cmd[n] = s[n]; n++; }
  cmd[n] = 0;
  const char *rest = s + n;
  while (*rest == ' ') rest++;

  if (!strcmp(cmd, "x")) { releaseAll(); return; }
  if (!strcmp(cmd, "e")) { engageAll(); return; }
  if (!strcmp(cmd, "osc")) {                 // генератор PCA9685, spotty_osc.h
    if (spottyOscCommand(rest, pca, engaged, Serial)) applyAll();
    return;
  }
  if (!strcmp(cmd, "s")) { status(); return; }
  if (!strcmp(cmd, "?") || !strcmp(cmd, "h")) { help(); return; }

  if (!strcmp(cmd, "o") || !strcmp(cmd, "n")) {
    long ch = atol(rest);
    if (!*rest || ch < 0 || ch > 15) { Serial.println(F("нужен канал 0..15")); return; }
    held[ch] = (cmd[0] == 'n');
    if (held[ch]) { if (engaged) spottyWriteUs(pca, (uint8_t)ch, clampCh((uint8_t)ch, targetUs[ch]));
                    Serial.printf("канал %ld снова удерживается на %u мкс\n", ch, targetUs[ch]); }
    else          { chOff((uint8_t)ch);
                    Serial.printf("канал %ld ОТПУЩЕН, стал мягким — подопри деталь\n", ch); }
    return;
  }

  if (!strcmp(cmd, "c")) {
    long ch = atol(rest);
    if (!*rest || ch < 0 || ch > 15) { Serial.println(F("c <0..15>")); return; }
    selCh = (int8_t)ch;
    Serial.printf("подстраивается канал %ld, цель %u мкс, шаг %u мкс\n",
                  ch, targetUs[ch], stepUs);
    Serial.println(F("'+' и '-' двигают, 'r' вернёт исходное, 'w' покажет итог"));
    return;
  }
  if (!strcmp(cmd, "+") || !strcmp(cmd, "-")) {
    if (selCh < 0) { Serial.println(F("сначала 'c <ch>'")); return; }
    int32_t t = (int32_t)targetUs[selCh] + (cmd[0] == '+' ? stepUs : -(int32_t)stepUs);
    uint16_t v = clampCh((uint8_t)selCh, t);
    if ((int32_t)v != t)
      Serial.printf("[!] упёрлись в жёсткий предел канала %d (%u..%u; измерено %u..%u, запас %.1f°)\n",
                    selCh, spottyHardLoUs((uint8_t)selCh), spottyHardHiUs((uint8_t)selCh),
                    JOINT_MIN_US[selCh], JOINT_MAX_US[selCh], SPOTTY_LIMIT_MARGIN_DEG);
    targetUs[selCh] = v;
    held[selCh] = true;
    if (engaged) spottyWriteUs(pca, (uint8_t)selCh, v);
    int d = (int)v - (int)origUs[selCh];
    Serial.printf("ch%d: %u мкс   (%+d от исходного = %+.1f град)%s\n",
                  selCh, v, d, d / 5.57f,
                  "");
    offcenterReport((uint8_t)selCh, v);
    return;
  }
  if (!strcmp(cmd, "r")) {
    if (selCh < 0) { Serial.println(F("сначала 'c <ch>'")); return; }
    targetUs[selCh] = origUs[selCh];
    if (engaged && held[selCh]) spottyWriteUs(pca, (uint8_t)selCh, targetUs[selCh]);
    Serial.printf("ch%d возвращён к исходному %u мкс\n", selCh, targetUs[selCh]);
    return;
  }
  if (!strcmp(cmd, "ss")) {
    long v = atol(rest);
    if (!*rest || v < 1 || v > 100) { Serial.println(F("ss <1..100>")); return; }
    stepUs = (uint16_t)v;
    Serial.printf("шаг = %u мкс = %.2f град\n", stepUs, stepUs / 5.57f);
    return;
  }
  if (!strcmp(cmd, "w"))   { exportDeltas(); return; }
  if (!strcmp(cmd, "nvs")) { nvsSave(); return; }
  if (!strcmp(cmd, "ld"))  { nvsLoad(false); applyAll(); return; }
  if (!strcmp(cmd, "clr")) { nvsClear(); return; }

  if (!strcmp(cmd, "p")) {
    long ch = 0, us = 0;
    if (sscanf(rest, "%ld %ld", &ch, &us) != 2 || ch < 0 || ch > 15) {
      Serial.println(F("p <ch> <us>")); return;
    }
    uint16_t t = clampCh((uint8_t)ch, us);
    if ((long)t != us) Serial.printf("[!] %ld за жёстким пределом канала %ld (%u..%u), ограничено до %u\n",
                                     us, ch, spottyHardLoUs((uint8_t)ch), spottyHardHiUs((uint8_t)ch), t);
    targetUs[ch] = t; held[ch] = true;
    if (engaged) spottyWriteUs(pca, (uint8_t)ch, t);
    Serial.printf("канал %ld удерживается на %u мкс\n", ch, t);
    Serial.println(F("Значение живёт только до перезагрузки: правь таблицу в скетче."));
    return;
  }

  Serial.println(F("неизвестная команда, '?' — справка"));
}

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    lastRxMs = millis();
    if (c == '\r' || c == '\n') { Serial.println();
      if (lineLen) { line[lineLen] = 0; handleLine(line); lineLen = 0; } continue; }
    Serial.write(c);
    if (lineLen < sizeof(line) - 1) line[lineLen++] = c;
  }
  if (lineLen && (millis() - lastRxMs) >= LINE_IDLE_MS) {
    line[lineLen] = 0; Serial.println(); handleLine(line); lineLen = 0;
  }
}

// -------------------------------- setup ------------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_HZ);
  buildTargets();

  Serial.println(F("\n===================================================="));
  Serial.println(F(" Spotty — servo_hold: удержание позы на время сборки"));
  Serial.println(F("====================================================")); 

#if defined(ARDUINO_ARCH_ESP32)
  {
    esp_reset_reason_t rr = esp_reset_reason();
    Serial.printf("причина перезагрузки MCU: %d%s\n", (int)rr,
                  rr == ESP_RST_BROWNOUT ? "  <- BROWNOUT, просадка питания MCU" : "");
  }
#endif

  initDriver();
  for (uint8_t ch = 0; ch < 16; ch++) chOff(ch);   // старт всегда с мягкого
  spottyOscBoot(pca, Serial);   // генератор PCA9685: до первой подачи ШИМ

  // Жёсткие пределы: самопроверка таблиц и сверка с локальной копией.
  // Обе печати обязательны перед автозахватом: оператор должен видеть,
  // в какое окно его сейчас загонят.
  Serial.println(F("\n--- жёсткие пределы каналов ---"));
  bool limitsOk = spottyLimitsSelfCheck(true);
  configDiffReport();

  Serial.println(F("\nЦели удержания (нули из spotty_config.h):"));
  uint8_t nProv = 0;
  for (uint8_t ch = 0; ch < 16; ch++) {
    if (!spottyChUsed(ch)) continue;
    Serial.printf("  ch%-3u %u мкс  окно %u..%u  %-20s %-7s %s\n", ch, targetUs[ch],
                  spottyHardLoUs(ch), spottyHardHiUs(ch),
                  TABLE[ch].leg, jointName(TABLE[ch].joint),
                  TABLE[ch].measured ? "предел измерен" : "предел ВРЕМЕННЫЙ");
    if (!TABLE[ch].measured) nProv++;
  }
  if (nProv) Serial.printf("\n[!] У %u суставов пределы ВРЕМЕННЫЕ.\n", nProv);
  else       Serial.println(F("\nВсе пределы измерены физически."));
  Serial.printf("центр рабочего окна: %u мкс\n", WINDOW_CENTER_US);
  nvsLoad(true);        // если во флеше есть подстроенные цели — они главнее

  Serial.println(F("\nВНИМАНИЕ. Сейчас будет подан ШИМ. Если деталь стоит НЕ там,"));
  Serial.println(F("куда её зовут, серво рванёт туда на полном моменте."));
  Serial.printf ("Любая клавиша в течение %u с — отменить автозахват.\n", ARM_SECONDS);

  bool aborted = false;
  if (!limitsOk) {
    aborted = true;
    Serial.println(F("\n[!!] САМОПРОВЕРКА ПРЕДЕЛОВ НЕ ПРОЙДЕНА — автозахват запрещён."));
    Serial.println(F("     Правь таблицы в spotty_config.h. 'e' включит вручную,"));
    Serial.println(F("     но кламп при битой таблице защитой считать нельзя."));
  }
  for (uint8_t i = ARM_SECONDS; i > 0 && !aborted; i--) {
    Serial.printf("  %u...\n", i);
    uint32_t t = millis();
    while (millis() - t < 1000) {
      if (Serial.available()) { while (Serial.available()) Serial.read(); aborted = true; break; }
      delay(10);
    }
  }

  if (aborted) {
    Serial.println(F("\nАвтозахват ОТМЕНЁН. Всё мягкое. 'e' — включить вручную."));
  } else {
    Serial.println();
    engageAll();
  }

  help();
  status();
  lastCheck = millis();
  lastBeat  = millis();
}

// --------------------------------- loop ------------------------------------
void loop() {
  pollSerial();                       // Serial читается всегда, 'x' срабатывает мгновенно
  uint32_t now = millis();

  if ((now - lastCheck) >= CHECK_MS) { lastCheck = now; if (engaged) watchdog(); }
  // Генератор PCA9685: фоновое слежение, раз в 2 с. Сменилась K — переписать каналы.
  if (spottyOscTrackTick(pca, Serial) && engaged) applyAll();

  if ((now - lastBeat) >= BEAT_MS) {
    lastBeat = now;
    Serial.printf("[%lu с] удержание %s, восстановлений %lu\n",
                  (unsigned long)(now / 1000),
                  engaged ? "включено" : "СНЯТО",
                  (unsigned long)recoveries);
  }
}
