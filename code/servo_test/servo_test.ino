/*
 * servo_test.ino — bring-up и НАГРУЗОЧНЫЙ тест, проект Spotty (OpenQuadruped)
 * ------------------------------------------------------------------------
 * ЖЕЛЕЗО (заявление оператора, 2026-08-20):
 *   MCU       : ESP32-S3 (один)
 *   Драйвер   : PCA9685, I2C addr 0x40, SDA = GPIO8, SCL = GPIO9
 *   Серво     : 12 x QY3240MG HV
 *   Питание   : отдельная 2S LiPo 50C (7.4 В ном. / 8.4 В заряженная)
 *   Механика  : серво на столе, КАЧАЛКИ СНЯТЫ
 *
 * НАЗНАЧЕНИЕ
 *   1) Проверить, что все каналы живы (одновременный прогон).
 *   2) НАГРУЗОЧНЫЙ ТЕСТ ПЛАТЫ: все каналы работают параллельно, а скетч
 *      при этом непрерывно опрашивает PCA9685 по I2C и ловит момент,
 *      когда драйвер «отваливается».
 *
 * КАК ЛОВИТСЯ ОТКАЗ (это главное в нагрузочном режиме)
 *   PCA9685 при просадке питания логики сбрасывается. После сброса:
 *     - регистр PRE_SCALE возвращается к заводскому значению;
 *     - в MODE1 поднимается бит SLEEP (0x10).
 *   Скетч запоминает PRE_SCALE сразу после инициализации и раз в 500 мс
 *   перечитывает оба регистра. Расхождение = зафиксированный сброс
 *   драйвера, тест немедленно останавливается. Отдельно считаются NACK/
 *   таймауты на шине — это уже деградация связи, а не сброс.
 *   Сброс самого ESP32-S3 виден по причине перезагрузки, которая
 *   печатается при старте.
 *
 * ЧЕГО СКЕТЧ НЕ МОЖЕТ
 *   В железе нет ни ADC на линии питания, ни датчика тока (HARDWARE.md, п.3).
 *   Напряжение на V+ PCA9685 под нагрузкой измеряется ТОЛЬКО мультиметром.
 *   Скетч видит последствия просадки, но не саму просадку.
 *
 * БЕЗОПАСНОСТЬ
 *   - Никакого движения при старте (SAFETY.md, правило 1): после загрузки
 *     все 16 каналов обесточены.
 *   - Движение неблокирующее: 'x' (стоп) срабатывает мгновенно, в том
 *     числе во время прогона. В историческом коде стоял delay() —
 *     программного останова там не было вообще.
 *   - Жёсткий кламп импульса 1000..2000 мкс. Исторические SERVO_MIN/MAX
 *     100..500 тиков (~488..2441 мкс) НЕ применяются: они записаны для
 *     MG996R, для QY3240MG HV не проверены (CALIBRATION.md, п. 1.7).
 *   - У нагрузочного прогона есть лимит длительности (по умолчанию 30 с).
 *
 * ЧТО ЗДЕСЬ НЕ ИЗМЕРЕНО (не выдавать за факт)
 *   - 1500 мкс как «центр» — предположение.
 *   - 50 Гц — кандидат из исторического кода.
 *   - Соответствие «канал PCA9685 -> нога/сустав» НЕ ОПРЕДЕЛЕНО.
 *     Скетч оперирует НОМЕРАМИ КАНАЛОВ, а не именами суставов.
 *
 * БИБЛИОТЕКА: Adafruit PWM Servo Driver Library 3.0.2 (проверено)
 * ПОРТ: Serial 115200
 * ------------------------------------------------------------------------
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#if defined(ARDUINO_ARCH_ESP32)
  #include <esp_system.h>
#endif

// ------------------------- конфигурация железа -------------------------
static const uint8_t  I2C_SDA_PIN = 8;
static const uint8_t  I2C_SCL_PIN = 9;
static const uint8_t  PCA_ADDR    = 0x40;
static const uint32_t I2C_HZ      = 400000UL;
static const float    PWM_HZ      = 50.0f;      // КАНДИДАТ, не измерение

// Регистры PCA9685.
// ВАЖНО: имена с префиксом PCAREG_/PCABIT_ намеренно.
// Adafruit_PWMServoDriver.h (>=3.0) уже определяет МАКРОСЫ MODE1_SLEEP,
// PCA9685_MODE1, PCA9685_PRESCALE и др. Своя переменная с таким же именем
// подставляется препроцессором в число и даёт
// "error: expected unqualified-id before numeric constant".
static const uint8_t PCAREG_MODE1    = 0x00;
static const uint8_t PCAREG_PRESCALE = 0xFE;
static const uint8_t PCABIT_SLEEP    = 0x10;   // MODE1 bit4 = SLEEP

// --------------------- жёсткие границы импульса ------------------------
static const uint16_t US_HARD_MIN = 1000;
static const uint16_t US_HARD_MAX = 2000;

// ------------------------ параметры прогона ----------------------------
static uint16_t usCenter = 1500;   // предполагаемая нейтраль
static uint16_t usAmp    = 150;    // амплитуда, мкс
static uint32_t periodMs = 3000;   // период синуса
static uint32_t stepMs   = 400;    // полупериод «шага» (реверс)
static uint32_t stagMs   = 0;      // разнос старта каналов, мс (0 = строго синхронно)
static uint32_t durS     = 30;     // лимит нагрузочного прогона, с (0 = без лимита)
static bool     waveStep = false;  // false = синус, true = шаг/реверс
static uint16_t chMask   = 0xFFFF; // какие каналы участвуют (бит = канал)

// Кандидат-набор каналов из исторической прошивки (CALIBRATION.md 1.1).
// Именно КАНДИДАТ: соответствие текущей разводке не проверено.
static const uint16_t MASK_CAND = (1u<<1)|(1u<<2)|(1u<<3)|(1u<<5)|(1u<<6)|(1u<<7)
                                |(1u<<9)|(1u<<10)|(1u<<11)|(1u<<13)|(1u<<14)|(1u<<15);
static const uint16_t MASK_LOW  = 0x0FFF;   // каналы 0..11
static const uint16_t MASK_ALL  = 0xFFFF;   // каналы 0..15

Adafruit_PWMServoDriver pca(PCA_ADDR);

enum Mode : uint8_t { M_OFF, M_HOLD, M_RUN };
static Mode     mode       = M_OFF;
static bool     telemetry  = false;   // true в нагрузочном режиме
static uint16_t holdUs     = 1500;
static uint32_t runT0      = 0;
static uint32_t lastUpdate = 0;
static uint32_t lastHealth = 0;
static uint32_t lastReport = 0;
static const uint32_t UPDATE_MS  = 20;    // 50 Гц обновление позиции
static const uint32_t HEALTH_MS  = 500;   // опрос регистров драйвера
static const uint32_t REPORT_MS  = 2000;  // печать отчёта

// --------------------------- здоровье драйвера -------------------------
static uint8_t  refPrescale = 0;      // эталон, снятый после инициализации
static bool     refValid    = false;
static uint32_t i2cOk       = 0;
static uint32_t i2cErr      = 0;
static uint32_t i2cErrRun   = 0;      // подряд идущие ошибки
static uint32_t resetSeen   = 0;
static uint32_t updates     = 0;
static const uint32_t I2C_ERR_ABORT = 5;   // подряд ошибок -> аварийный стоп

static char    line[64];
static uint8_t lineLen = 0;

// --- приём команд ---
// Команда исполняется по '\n', ПО '\r' И по паузе в приёме.
// Пауза нужна для режима монитора "No Line Ending" / "Без конца строки":
// там терминатор не отправляется вообще, и парсер, ждущий только '\n',
// молчит на любую команду. Именно так выглядит "монитор не реагирует".
static uint32_t rxTotal   = 0;      // всего принято байт с момента старта
static uint32_t lastRxMs  = 0;
static bool     echoOn    = true;   // эхо ввода — видно, доходят ли символы
static uint32_t lastBeat  = 0;
static const uint32_t LINE_IDLE_MS = 120;   // пауза = конец команды
static const uint32_t BEAT_MS      = 5000;  // период маячка простоя

// ----------------------------- утилиты ---------------------------------
static uint16_t clampUs(int32_t us) {
  if (us < (int32_t)US_HARD_MIN) return US_HARD_MIN;
  if (us > (int32_t)US_HARD_MAX) return US_HARD_MAX;
  return (uint16_t)us;
}

static void chOff(uint8_t ch) { pca.setPWM(ch, 0, 4096); }  // full-off

static void allOff() {
  for (uint8_t ch = 0; ch < 16; ch++) chOff(ch);
  mode = M_OFF;
  telemetry = false;
}

static uint8_t maskCount() {
  uint8_t n = 0;
  for (uint8_t ch = 0; ch < 16; ch++) if (chMask & (1u << ch)) n++;
  return n;
}

static void writeMasked(uint16_t us) {
  us = clampUs(us);
  for (uint8_t ch = 0; ch < 16; ch++)
    if (chMask & (1u << ch)) pca.writeMicroseconds(ch, us);
}

static void printMask() {
  Serial.print(F("каналы: "));
  for (uint8_t ch = 0; ch < 16; ch++)
    if (chMask & (1u << ch)) { Serial.print(ch); Serial.print(' '); }
  Serial.print(F(" (всего "));
  Serial.print(maskCount());
  Serial.println(F(")"));
}

// ------------------------- низкоуровневый I2C --------------------------
static bool pcaRead(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)PCA_ADDR, 1) != 1) return false;
  val = (uint8_t)Wire.read();
  return true;
}

// Возвращает true, если драйвер здоров. Печатает диагноз при отказе.
static bool healthCheck() {
  uint8_t ps = 0, m1 = 0;
  bool okPs = pcaRead(PCAREG_PRESCALE, ps);
  bool okM1 = pcaRead(PCAREG_MODE1, m1);

  if (!okPs || !okM1) {
    i2cErr++; i2cErrRun++;
    Serial.printf("[!] I2C: чтение регистров не удалось (подряд %lu)\n",
                  (unsigned long)i2cErrRun);
    if (i2cErrRun >= I2C_ERR_ABORT) {
      Serial.println(F("[!!] Шина I2C не отвечает. Причины по убыванию вероятности:"));
      Serial.println(F("     просадка/помеха по питанию логики PCA9685; общая земля;"));
      Serial.println(F("     наводка от силовых проводов серво на SDA/SCL."));
      return false;
    }
    return true;   // одиночный сбой — не повод рвать тест
  }

  i2cOk++; i2cErrRun = 0;

  if (refValid && ps != refPrescale) {
    resetSeen++;
    Serial.printf("[!!] PCA9685 СБРОСИЛСЯ: PRE_SCALE %u -> %u\n", refPrescale, ps);
    Serial.println(F("     Драйвер потерял питание логики под нагрузкой."));
    return false;
  }
  if (m1 & PCABIT_SLEEP) {
    resetSeen++;
    Serial.printf("[!!] PCA9685 СБРОСИЛСЯ: MODE1=0x%02X, поднят бит SLEEP\n", m1);
    Serial.println(F("     Драйвер потерял питание логики под нагрузкой."));
    return false;
  }
  return true;
}

// ------------------------------ движение -------------------------------
static void motionUpdate(uint32_t now) {
  uint8_t idx = 0;
  for (uint8_t ch = 0; ch < 16; ch++) {
    if (!(chMask & (1u << ch))) continue;
    uint32_t t = (now - runT0) + (uint32_t)idx * stagMs;
    int32_t us;
    if (waveStep) {
      us = ((t / stepMs) & 1UL) ? (int32_t)usCenter + usAmp
                                : (int32_t)usCenter - usAmp;
    } else {
      float ph = (float)(t % periodMs) / (float)periodMs;
      us = (int32_t)(usCenter + usAmp * sinf(ph * 2.0f * (float)M_PI));
    }
    pca.writeMicroseconds(ch, clampUs(us));
    idx++;
  }
  updates++;
}

static void report(uint32_t now) {
  uint32_t el = now - runT0;
  Serial.printf("[t=%6.1f s] кан=%u  обновл=%lu  I2C ok=%lu err=%lu  сбросов=%lu  %s\n",
                el / 1000.0f, maskCount(), (unsigned long)updates,
                (unsigned long)i2cOk, (unsigned long)i2cErr,
                (unsigned long)resetSeen,
                resetSeen ? "ОТКАЗ" : "драйвер жив");
}

static void summary(const char *why) {
  Serial.println(F("--- итог прогона ---"));
  Serial.printf("причина останова : %s\n", why);
  Serial.printf("каналов          : %u\n", maskCount());
  Serial.printf("обновлений позиц.: %lu\n", (unsigned long)updates);
  Serial.printf("I2C ok / ошибок  : %lu / %lu\n",
                (unsigned long)i2cOk, (unsigned long)i2cErr);
  Serial.printf("сбросов драйвера : %lu\n", (unsigned long)resetSeen);
  if (resetSeen == 0 && i2cErr == 0)
    Serial.println(F("ВЕРДИКТ: плата держит все каналы параллельно, связь не рвалась."));
  else
    Serial.println(F("ВЕРДИКТ: зафиксирован отказ. Мерить V+ мультиметром под нагрузкой."));
  Serial.println(F("Все каналы обесточены."));
}

static void startRun(bool withTelemetry) {
  updates = 0; i2cOk = 0; i2cErr = 0; i2cErrRun = 0; resetSeen = 0;
  runT0 = millis(); lastUpdate = 0; lastHealth = runT0; lastReport = runT0;
  telemetry = withTelemetry;
  mode = M_RUN;
  Serial.printf("СТАРТ %s: %u +/- %u мкс, %s, каналов %u, разнос %lu мс",
                withTelemetry ? "нагрузочного теста" : "прогона",
                usCenter, usAmp,
                waveStep ? "ШАГ/реверс" : "синус",
                maskCount(), (unsigned long)stagMs);
  if (withTelemetry && durS) Serial.printf(", лимит %lu с", (unsigned long)durS);
  Serial.println();
  Serial.println(F("Стоп в любой момент: 'x'"));
}

// Подсказка после команд, которые только МЕНЯЮТ ПАРАМЕТР и сами по себе
// ничего не крутят. Без неё "wave step" выглядит как проигнорированная команда.
static void paramNote() {
  if (mode == M_RUN) Serial.println(F("  (прогон идёт — применено на лету)"));
  else               Serial.println(F("  (это параметр, движения нет — запусти 'l' или 'a')"));
}

// ------------------------------ справка --------------------------------
static void status() {
  Serial.println(F("--- состояние ---"));
  Serial.print(F("режим    : "));
  Serial.println(mode == M_OFF ? F("OFF (серво обесточены)")
               : mode == M_HOLD ? F("HOLD (удержание)")
                                : F("RUN (движение)"));
  Serial.printf("форма    : %s   центр %u мкс   ампл %u мкс\n",
                waveStep ? "ШАГ/реверс" : "синус", usCenter, usAmp);
  Serial.printf("период   : синус %lu мс / шаг %lu мс   разнос %lu мс\n",
                (unsigned long)periodMs, (unsigned long)stepMs, (unsigned long)stagMs);
  Serial.printf("кламп    : %u..%u мкс (жёсткий)\n", US_HARD_MIN, US_HARD_MAX);
  Serial.printf("PWM      : %.0f Гц (кандидат), PRE_SCALE эталон %u\n",
                PWM_HZ, refPrescale);
  Serial.printf("лимит    : %lu с\n", (unsigned long)durS);
  printMask();
}

static void help() {
  Serial.println(F(
    "\n=== servo_test — команды ===\n"
    "  x            СТОП: снять ШИМ со всех каналов (серво мягкие)\n"
    "  i            скан шины I2C\n"
    "  st           состояние\n"
    "  h            справка\n"
    "  echo on/off  эхо вводимых символов (по умолчанию ВКЛ)\n"
    "  --- прогон (все каналы маски ПАРАЛЛЕЛЬНО) ---\n"
    "  l            НАГРУЗОЧНЫЙ ТЕСТ: движение + опрос драйвера + отчёты\n"
    "  a            просто движение, без телеметрии\n"
    "  c            встать в центр и удерживать\n"
    "  w <us>       удерживать все каналы на <us> мкс\n"
    "  s <ch>       прогон ТОЛЬКО канала <ch>\n"
    "  --- параметры (только настройка, движение не запускают) ---\n"
    "  wave sine    плавный синус\n"
    "  wave step    шаг/реверс — худший случай по току\n"
    "  cen <us>     центр, мкс (1500)\n"
    "  amp <us>     амплитуда, мкс (150, максимум 400)\n"
    "  per <ms>     период синуса, мс (3000)\n"
    "  step <ms>    полупериод шага, мс (400)\n"
    "  stag <ms>    разнос старта каналов, мс (0 = строго синхронно)\n"
    "  dur <s>      лимит нагрузочного прогона, с (30; 0 = без лимита)\n"
    "  m all        маска = каналы 0..15\n"
    "  m low        маска = каналы 0..11\n"
    "  m cand       маска = кандидат {1,2,3,5,6,7,9,10,11,13,14,15}\n"
    "  m <ch>       маска = только канал <ch>\n"
  ));
}

static void i2cScan() {
  Serial.println(F("I2C scan (SDA=8, SCL=9):"));
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X%s\n", a,
                    (a == PCA_ADDR) ? "   <- PCA9685, ожидаемый адрес" : "");
      found++;
    }
  }
  if (!found)
    Serial.println(F("  НИЧЕГО НЕ НАЙДЕНО. Проверь: SDA/SCL не перепутаны,\n"
                     "  подано ли питание на ЛОГИКУ PCA9685 (VCC, не V+),\n"
                     "  есть ли общая земля MCU <-> PCA9685."));
}

// ---------------------------- разбор команд ----------------------------
static void handleLine(char *s) {
  while (*s == ' ') s++;
  if (!*s) return;

  char cmd[8] = {0};
  long arg = 0;
  bool hasArg = false;
  int n = 0;
  while (s[n] && s[n] != ' ' && n < 7) { cmd[n] = s[n]; n++; }
  cmd[n] = 0;
  const char *rest = s + n;
  while (*rest == ' ') rest++;

  if (*rest) {
    if (!strcmp(cmd, "m")) {
      if (!strcmp(rest, "all"))  { chMask = MASK_ALL;  printMask(); return; }
      if (!strcmp(rest, "low"))  { chMask = MASK_LOW;  printMask(); return; }
      if (!strcmp(rest, "cand")) { chMask = MASK_CAND; printMask(); return; }
    }
    if (!strcmp(cmd, "echo")) {
      if (!strcmp(rest, "on"))  { echoOn = true;  Serial.println(F("эхо ввода: ВКЛ")); return; }
      if (!strcmp(rest, "off")) { echoOn = false; Serial.println(F("эхо ввода: ВЫКЛ")); return; }
      Serial.println(F("echo on | echo off")); return;
    }
    if (!strcmp(cmd, "wave")) {
      if (!strcmp(rest, "sine")) { waveStep = false; Serial.println(F("форма: синус")); paramNote(); return; }
      if (!strcmp(rest, "step")) { waveStep = true;  Serial.println(F("форма: ШАГ/реверс")); paramNote(); return; }
      Serial.println(F("wave sine | wave step")); return;
    }
    arg = atol(rest);
    hasArg = true;
  }

  if (!strcmp(cmd, "x")) {
    bool wasRun = (mode == M_RUN);
    bool tel = telemetry;
    allOff();
    if (wasRun && tel) summary("останов оператором");
    else Serial.println(F("СТОП. Все каналы обесточены."));
    return;
  }
  if (!strcmp(cmd, "h") || !strcmp(cmd, "?")) { help(); return; }
  if (!strcmp(cmd, "i"))  { i2cScan(); return; }
  if (!strcmp(cmd, "st")) { status();  return; }

  if (!strcmp(cmd, "l")) { startRun(true);  return; }
  if (!strcmp(cmd, "a")) { startRun(false); return; }

  if (!strcmp(cmd, "c")) {
    holdUs = usCenter; mode = M_HOLD; telemetry = false; writeMasked(holdUs);
    Serial.printf("HOLD %u мкс на %u каналах\n", holdUs, maskCount());
    return;
  }
  if (!strcmp(cmd, "w") && hasArg) {
    holdUs = clampUs(arg); mode = M_HOLD; telemetry = false; writeMasked(holdUs);
    Serial.printf("HOLD %u мкс на %u каналах\n", holdUs, maskCount());
    return;
  }
  if (!strcmp(cmd, "s") && hasArg) {
    if (arg < 0 || arg > 15) { Serial.println(F("канал 0..15")); return; }
    allOff();
    chMask = (uint16_t)(1u << (uint8_t)arg);
    startRun(false);
    return;
  }
  if (!strcmp(cmd, "cen") && hasArg) { usCenter = clampUs(arg); Serial.printf("центр = %u мкс\n", usCenter); paramNote(); return; }
  if (!strcmp(cmd, "amp") && hasArg) {
    if (arg < 0) arg = 0;
    if (arg > 400) arg = 400;
    usAmp = (uint16_t)arg; Serial.printf("амплитуда = %u мкс\n", usAmp); paramNote(); return;
  }
  if (!strcmp(cmd, "per") && hasArg) {
    if (arg < 300) arg = 300;
    periodMs = (uint32_t)arg; Serial.printf("период синуса = %lu мс\n", (unsigned long)periodMs); paramNote(); return;
  }
  if (!strcmp(cmd, "step") && hasArg) {
    if (arg < 100) arg = 100;
    stepMs = (uint32_t)arg; Serial.printf("полупериод шага = %lu мс\n", (unsigned long)stepMs); paramNote(); return;
  }
  if (!strcmp(cmd, "stag") && hasArg) {
    if (arg < 0) arg = 0;
    if (arg > 500) arg = 500;
    stagMs = (uint32_t)arg; Serial.printf("разнос каналов = %lu мс\n", (unsigned long)stagMs); paramNote(); return;
  }
  if (!strcmp(cmd, "dur") && hasArg) {
    if (arg < 0) arg = 0;
    durS = (uint32_t)arg; Serial.printf("лимит = %lu с\n", (unsigned long)durS); return;
  }
  if (!strcmp(cmd, "m") && hasArg) {
    if (arg < 0 || arg > 15) { Serial.println(F("канал 0..15")); return; }
    chMask = (uint16_t)(1u << (uint8_t)arg); printMask(); return;
  }

  Serial.println(F("неизвестная команда, 'h' — справка"));
}

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    rxTotal++;
    lastRxMs = millis();

    if (echoOn) {                       // эхо: видно, что символ реально дошёл
      if (c == '\r' || c == '\n') Serial.println();
      else                        Serial.write(c);
    }

    if (c == '\r' || c == '\n') {       // терминатор любого вида
      if (lineLen) { line[lineLen] = 0; handleLine(line); lineLen = 0; }
      continue;                         // "\r\n" -> второй символ просто пустая строка
    }
    if (lineLen < sizeof(line) - 1) line[lineLen++] = c;
  }

  // Монитор без конца строки: терминатор не придёт никогда — режем по паузе.
  if (lineLen && (millis() - lastRxMs) >= LINE_IDLE_MS) {
    line[lineLen] = 0;
    if (echoOn) Serial.println();
    handleLine(line);
    lineLen = 0;
  }
}

// Маячок простоя. Главное здесь — счётчик принятых байт:
// он отделяет "команда не разобралась" от "приём вообще мёртв".
static void beat() {
  Serial.printf("[ожидание команды]  принято байт: %lu%s\n",
                (unsigned long)rxTotal,
                rxTotal ? "" : "  <- НОЛЬ: символы до платы не доходят");
}

// -------------------------------- setup --------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { /* ждём USB-CDC, но не вечно */ }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_HZ);

  Serial.println(F("\n===================================================="));
  Serial.println(F(" Spotty / OpenQuadruped — servo_test"));
  Serial.println(F(" ESP32-S3 + PCA9685 @0x40 + 12 x QY3240MG HV"));
  Serial.println(F("====================================================")); 

#if defined(ARDUINO_ARCH_ESP32)
  {
    esp_reset_reason_t rr = esp_reset_reason();
    Serial.printf("причина перезагрузки MCU: %d", (int)rr);
    if (rr == ESP_RST_BROWNOUT)      Serial.print(F("  <- BROWNOUT: просадка питания MCU в прошлом прогоне"));
    else if (rr == ESP_RST_PANIC)    Serial.print(F("  <- PANIC"));
    else if (rr == ESP_RST_POWERON)  Serial.print(F("  (обычное включение)"));
    Serial.println();
  }
#endif

  i2cScan();

  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(PWM_HZ);
  delay(10);

  if (pcaRead(PCAREG_PRESCALE, refPrescale)) {
    refValid = true;
    Serial.printf("PRE_SCALE после инициализации: %u (эталон для детекта сброса)\n", refPrescale);
  } else {
    Serial.println(F("[!] Не удалось прочитать PRE_SCALE — детект сброса драйвера ОТКЛЮЧЁН."));
  }

  allOff();   // ВАЖНО: стартуем без движения, все каналы обесточены

  Serial.println(F("\nЕсли команды не срабатывают — проверь в мониторе поле конца строки."));
  Serial.println(F("Этот скетч принимает любой вариант: NL, CR, NL+CR и даже 'без конца строки'."));
  Serial.println(F("Вводимые символы отражаются эхом. Эха нет = приём не работает."));
  Serial.println(F("\nВсе каналы обесточены. Движения при старте НЕТ."));
  Serial.println(F("Аварийный останов: 'x' + физический разрыв питания серво."));
  help();
  status();
}

// -------------------------------- loop ---------------------------------
void loop() {
  pollSerial();                 // Serial читается ВСЕГДА, в том числе во время прогона

  uint32_t now = millis();

  if (mode != M_RUN) {
    if ((now - lastBeat) >= BEAT_MS) { lastBeat = now; beat(); }
    return;
  }

  if ((now - lastUpdate) >= UPDATE_MS) { lastUpdate = now; motionUpdate(now); }

  if (!telemetry) return;

  if ((now - lastHealth) >= HEALTH_MS) {
    lastHealth = now;
    if (!healthCheck()) { allOff(); summary("ОТКАЗ ДРАЙВЕРА / ШИНЫ"); return; }
  }

  if ((now - lastReport) >= REPORT_MS) { lastReport = now; report(now); }

  if (durS && (now - runT0) >= durS * 1000UL) {
    allOff();
    summary("лимит времени исчерпан");
  }
}
