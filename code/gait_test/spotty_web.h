#pragma once
/*
 * spotty_web.h — веб-пульт и телеметрия для gait_test.
 * ESP32-S3, режим STA. Добавлено 2026-08-31.
 *
 * ===========================================================================
 * ГЛАВНОЕ СВОЙСТВО, РАДИ КОТОРОГО ВСЁ ПОСТРОЕНО ИМЕННО ТАК
 * ---------------------------------------------------------------------------
 * ВЕБ НЕ ДОБАВЛЯЕТ НИ ОДНОГО НОВОГО ПУТИ К СЕРВО.
 *
 * Браузер умеет ровно одно: положить строку в очередь. Строку разбирает тот же
 * handleLine(), на том же ядре, в том же потоке, что и строку из USB. Значит
 * предполётная проверка, paramGuard, жёсткий кламп и аварийный стоп работают
 * ровно так же и обойти их из браузера нельзя — их просто нечем обойти.
 *
 * Отсюда все решения ниже:
 *   - команды НЕ исполняются в задаче веб-сервера. Они кладутся в очередь,
 *     а исполняет их loop() на ядре 1, вызывая spottyWebPump();
 *   - телеметрия НЕ читается веб-задачей из живых переменных. loop() публикует
 *     снимок через seqlock, веб-задача читает копию. Иначе на ядре 0 можно
 *     было бы поймать угол, наполовину обновлённый ядром 1;
 *   - OTA НЕТ и не планируется в этой ревизии. См. «ЧЕГО ЗДЕСЬ НЕТ».
 *
 * ===========================================================================
 * РАЗДЕЛЕНИЕ ПО ЯДРАМ
 * ---------------------------------------------------------------------------
 * Тик походки 3 мс. Wi-Fi-стек ESP32 живёт на ЯДРЕ 0, Arduino loop() — на
 * ЯДРЕ 1. Поэтому loop() не трогаем вовсе: веб-сервер уезжает отдельной
 * задачей, ПРИБИТОЙ К ЯДРУ 0, и с тиком не конкурирует за процессор.
 *
 * Остаются два общих ресурса, и оба сделаны без блокировки ядра 1 на время
 * работы ядра 0:
 *   - снимок телеметрии: seqlock (счётчик версий, нечёт = идёт запись).
 *     Писатель никогда не ждёт, читатель перечитывает при коллизии;
 *   - лог: кольцевой буфер под коротким portENTER_CRITICAL. Критическая
 *     секция держится на время memcpy одной строки (десятки байт), а не на
 *     время HTTP-ответа.
 *
 * ЧЕСТНО О ЦЕНЕ. Радио всё равно даёт прерывания, а они прилетают на оба
 * ядра. Насколько это съедает тик — НЕ ПРЕДСКАЗАНО, а ИЗМЕРЯЕТСЯ: счётчик
 * tickOverruns уже есть в скетче и виден в панели. Смотреть на него при
 * первом же прогоне: если после включения веба он начал расти, а без веба
 * не рос — цену видно прямо в числе.
 *
 * ===========================================================================
 * WATCHDOG СВЯЗИ — ЗАЧЕМ И ПОЧЕМУ ИМЕННО 'x'
 * ---------------------------------------------------------------------------
 * Wi-Fi — это канал, который может пропасть молча: ушёл за стену, роутер
 * моргнул, браузер усыпил вкладку. Если робот в этот момент шагает, а пульта
 * больше нет — это ровно та ситуация, ради которой SAFETY.md требует разрыв
 * питания под рукой.
 *
 * Поэтому: пока роботом управляют ИЗ БРАУЗЕРА и он под током, отсутствие
 * запросов дольше SPOTTY_WEB_HB_MS считается потерей пульта, и скетч сам
 * останавливается. С 2026-09-27 — не заморозкой посреди шага, а доводя
 * действие до устойчивого конца (см. webLostRemote в gait_test.ino).
 *
 * Почему 'x' (удержание), а НЕ 'r' (снять ШИМ). 'r' обесточивает серво, и
 * стоящий робот от этого просто складывается. Потеря связи — не повод ронять
 * робота. 'x' замораживает текущее положение, то есть делает ровно то, что
 * сделал бы оператор, отпустив управление.
 *
 * Watchdog взводится ТОЛЬКО после команды из браузера и снимается на 'x'/'r'.
 * Работа через один USB, без единого захода в панель, ведёт себя как раньше:
 * watchdog спит и ничего не сторожит.
 *
 * ===========================================================================
 * ЧЕГО ЗДЕСЬ НЕТ И ПОЧЕМУ
 * ---------------------------------------------------------------------------
 *   - OTA (заливка прошивки по воздуху). Перезагрузка ESP32 не сбрасывает
 *     PCA9685: тот держит последний ШИМ, потом отдаёт непредсказуемое. Робот
 *     под током при этом падает. Делать OTA имеет смысл только с жёсткой
 *     блокировкой «нельзя, пока engaged», и это отдельная ревизия;
 *   - авторизации. В домашней сети её нет намеренно: пароль на пульте — это
 *     лишний экран между оператором и кнопкой СТОП. Следствие принять как
 *     есть: КТО В СЕТИ, ТОТ И УПРАВЛЯЕТ. Не поднимать веб в чужой сети;
 *   - WebSocket. Опрос по HTTP проще, не тянет внешних библиотек и не держит
 *     соединение, которое всё равно рвётся первым при уходе за стену.
 *     На 5 Гц телеметрии разницы нет;
 *   - записи параметров мимо handleLine. Ползунок в панели шлёт ту же строку
 *     't 12', что и клавиатура.
 * ===========================================================================
 */

#include <Arduino.h>
#include <stdarg.h>
#include <string.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "spotty_log.h"      // LOG и кольцо (включён уже до конфига)
#include "spotty_wifi.h"
#include "spotty_page.h"

// --------------------------- настройки -------------------------------------
static const uint16_t SPOTTY_WEB_PORT    = 80;
/*
 * Порог тишины до 'x'. ОБЯЗАН быть больше HTTP_MAX_DATA_WAIT ядра (5000 мс):
 * WebServer обслуживает ОДНО соединение за раз и держит занятого клиента до
 * этого таймаута. Порог меньше — и мы сторожим собственный веб-сервер, а не
 * связь, получая 'x' посреди прогона на ровном месте.
 */
// 2026-09-27: 6000 -> 12000 по решению оператора. На выставке Wi-Fi шумный,
// и 6 с тишины от одной помехи замораживали робота посреди действия.
// Что именно делается при потере — решает скетч (webLostRemote в gait_test).
static const uint32_t SPOTTY_WEB_HB_MS   = 12000;
static const uint32_t SPOTTY_WEB_CONN_MS = 20000;  // ждать сеть при старте
static const size_t   SPOTTY_LOG_CHUNK   = 1600;   // максимум за один запрос
static const uint8_t  SPOTTY_CMD_QUEUE   = 8;
static const size_t   SPOTTY_CMD_MAX     = 40;     // = sizeof(cmdLine) в скетче

// Состояние сети для печати из loop() (веб-задача сама в лог не пишет —
// у кольца ровно один писатель, и это ядро 1).
enum SpotNet : uint8_t { NET_OFF, NET_CONNECTING, NET_UP, NET_FAILED, NET_LOST, NET_AP };
volatile uint8_t spotNetState = NET_OFF;
volatile uint32_t spotReconn = 0;      // сколько раз переподключались к роутеру

/* СВОЯ СЕТЬ (точка доступа), 2026-09-29. Значения — в spotty_wifi.h; здесь
 * умолчания, чтобы старый spotty_wifi.h тоже собирался.
 *   SPOTTY_WIFI_MODE 0 — роутер; не нашёлся за 20 с -> своя сеть вместо
 *                        выключенного радио (раньше без роутера пульта не было);
 *   SPOTTY_WIFI_MODE 1 — только своя сеть: выставка, роутер не нужен.
 * Адрес в своей сети всегда 192.168.4.1. */
#ifndef SPOTTY_WIFI_MODE
#define SPOTTY_WIFI_MODE 0
#endif
#ifndef SPOTTY_AP_SSID
#define SPOTTY_AP_SSID "Spotty-Reboot"
#endif
#ifndef SPOTTY_AP_PASS
#define SPOTTY_AP_PASS "changeme123"
#endif
#ifndef SPOTTY_AP_CHANNEL
#define SPOTTY_AP_CHANNEL 6
#endif
static const uint32_t SPOTTY_WEB_RETRY_MS = 5000;   // повтор подключения к роутеру
char             spotNetIp[20] = "-";

// ===========================================================================
// СНИМОК ТЕЛЕМЕТРИИ — seqlock
// ---------------------------------------------------------------------------
// Пишет только ядро 1 (loop), читает только ядро 0 (веб). Писатель не ждёт
// никогда: это принципиально, тик 3 мс не может стоять в очереди за HTTP.
// ===========================================================================
struct SpotSnap {
  uint32_t ms;
  uint8_t  mode;
  uint8_t  gaitIdx;
  uint8_t  walkCycles;
  bool     engaged, havePose, tablesOk, speedRamping, flatPath, smoothSwing;
  int8_t   turn;
  float    phase, cyclesDone;
  float    cur[4][3];
  float    duty, ampThigh, ampKnee, cycleMs, ampHip;
  float    crouch, trim, linkRatio, maxRate, peakRate;
  uint32_t limitHits, tickOverruns, busUsLast, busUsMax;
  uint32_t freeHeap;
  // IMU (spotty_imu.h). Углы уже за вычетом 'imu zero'. ОСИ НЕ ПРОВЕРЕНЫ на
  // собранном роботе: крен + правый борт вниз, тангаж + нос вверх, курс + нос влево.
  bool     imuOk;
  float    imuRoll, imuPitch, imuYaw, imuA[3], imuStop;
  // Параметры декартова движка (pEngine == 1); в угловом только для справки.
  uint8_t  engine;
  float    cH, cP, cC, cA, cV, cW, cD, cX, cY, cR, cB, cPitch, cYaw;
  // Трюк (команда 'e'): номер или -1, кадр (0 пролог, 1..n, n+1 эпилог),
  // проверен ли знак хипа.
  int8_t   trick;
  uint8_t  trickKey;
  bool     trickHipOk;
  int8_t   prof;          // профиль ходьбы 'M' или -1 (2026-09-30)
};

static SpotSnap          spotSnap;
static volatile uint32_t spotSnapVer = 0;

// Вызывается из loop() на ядре 1.
static void spotSnapPublish(const SpotSnap &s) {
  spotSnapVer++;                  // нечёт: идёт запись
  __sync_synchronize();
  spotSnap = s;
  __sync_synchronize();
  spotSnapVer++;                  // чёт: снимок целостен
}

// Вызывается из веб-задачи на ядре 0. false = не удалось прочитать целостно.
static bool spotSnapRead(SpotSnap &out) {
  for (int try_ = 0; try_ < 8; try_++) {
    uint32_t v1 = spotSnapVer;
    if (v1 & 1) { delay(1); continue; }
    __sync_synchronize();
    out = spotSnap;
    __sync_synchronize();
    if (spotSnapVer == v1) return true;
  }
  return false;
}

// ===========================================================================
// ОЧЕРЕДЬ КОМАНД
// ===========================================================================
struct SpotCmd { char line[SPOTTY_CMD_MAX]; };
static QueueHandle_t     spotCmdQ = nullptr;
/*
 * Здесь ЖИЛ счётчик «есть что забрать», чтобы loop() не дёргал очередь
 * впустую. Убран 2026-08-31 по итогам вычитки: инкремент шёл с ядра 0,
 * декремент с ядра 1, а `volatile` атомарности не даёт — это load-add-store.
 * Потерянный инкремент означал бы, что команда лежит в очереди, а pump до
 * неё не доходит: МОЛЧА ПРОГЛОЧЕННЫЙ СТОП при отданном браузеру «ok».
 * Пустой xQueueReceive стоит единицы микросекунд при бюджете тика 3000 —
 * дешевле, чем городить когерентность ради этой экономии.
 */
static volatile uint32_t spotLastContactMs = 0;   // любой запрос от браузера
static volatile bool     spotWebControls   = false; // взведён ли watchdog

// ===========================================================================
// HTTP
// ===========================================================================
static WebServer spotServer(SPOTTY_WEB_PORT);

// Буферы ответов статические: они живут только в веб-задаче, и держать
// килобайты на её стеке незачем.
static char spotJson[3000];
static char spotRaw[SPOTTY_LOG_CHUNK + 8];
static char spotEsc[SPOTTY_LOG_CHUNK * 2 + 16];   // см. spotJsonEscape о вместимости

static const char *SPOT_MODE_NAME[] = { "IDLE", "ВЗВОД", "РАМПА", "ХОД", "УДЕРЖ" };

/*
 * Экранирование для JSON. Русский текст — это UTF-8, и он проходит в строку
 * JSON байт в байт. Опасны только управляющие символы, кавычка и обратный
 * слэш. Отдельно см. spotTrimUtf8 ниже.
 */
/*
 * Возвращает, сколько байт ИСХОДНИКА уместилось (не сколько записано в dst).
 * Это принципиально: вызывающий двигает смещение клиента на СЪЕДЕННОЕ. Если
 * бы он двигал его на n, а буфер вывода при этом переполнился, кусок лога
 * пропал бы МОЛЧА — клиент попросил бы уже следующий кусок и не узнал, что
 * между ними дырка. Управляющий символ раздувается вшестеро (\u0001), так
 * что переполнение в принципе достижимо, пусть и на мусорных данных.
 */
static size_t spotJsonEscape(const char *src, size_t n, char *dst, size_t cap,
                             size_t *usedSrc) {
  size_t o = 0, i = 0;
  for (i = 0; i < n && o + 8 < cap; i++) {
    unsigned char c = (unsigned char)src[i];
    switch (c) {
      case '"':  dst[o++] = '\\'; dst[o++] = '"';  break;
      case '\\': dst[o++] = '\\'; dst[o++] = '\\'; break;
      case '\n': dst[o++] = '\\'; dst[o++] = 'n';  break;
      case '\r': dst[o++] = '\\'; dst[o++] = 'r';  break;
      case '\t': dst[o++] = '\\'; dst[o++] = 't';  break;
      default:
        if (c < 0x20) { o += snprintf(dst + o, cap - o, "\\u%04X", c); }
        else          { dst[o++] = (char)c; }
    }
  }
  dst[o] = 0;
  if (usedSrc) *usedSrc = i;
  return o;
}

/*
 * ОБРЕЗКА ПО ГРАНИЦЕ UTF-8 — не косметика, а условие работоспособности.
 * Кольцо режется по байтам, а кириллица занимает два байта. Половина буквы
 * на конце куска даёт невалидный JSON, JSON.parse в браузере падает, и
 * панель молча перестаёт обновлять лог — то есть оператор перестаёт видеть
 * предупреждения ровно тогда, когда их больше всего.
 * Отбрасываем незаконченную последовательность в хвосте; следующий запрос
 * заберёт её целиком (смещение сдвинется только на отданное).
 */
static size_t spotTrimUtf8(const char *buf, size_t n) {
  if (!n) return 0;
  size_t cut = n;
  // Не более 3 шагов назад: длиннее 4 байт последовательностей в UTF-8 нет.
  for (int back = 0; back < 4 && cut > 0; back++) {
    unsigned char c = (unsigned char)buf[cut - 1];
    if ((c & 0x80) == 0x00) return cut;              // ASCII — граница здесь
    if ((c & 0xC0) == 0x80) { cut--; continue; }     // хвостовой байт — назад
    // Ведущий байт: длина последовательности
    size_t need = (c & 0xE0) == 0xC0 ? 2 : (c & 0xF0) == 0xE0 ? 3 : (c & 0xF8) == 0xF0 ? 4 : 1;
    size_t have = n - (cut - 1);
    return (have >= need) ? n : (cut - 1);
  }
  return cut;
}

static void spotTouch() { spotLastContactMs = millis(); }

static void spotSendJson(const char *body) {
  spotServer.sendHeader("Cache-Control", "no-store");
  spotServer.send(200, "application/json; charset=utf-8", body);
}

static void spotHandleRoot() {
  spotTouch();
  spotServer.sendHeader("Cache-Control", "no-store");
  spotServer.send_P(200, "text/html; charset=utf-8", SPOTTY_PAGE);
}

/*
 * Накопление в буфер БЕЗ шанса уехать за границу.
 * snprintf возвращает длину, которая была БЫ записана; наивное `o += snprintf`
 * при усечении уводит o за размер буфера, и следующий вызов получает
 * (size_t)(cap - o) — беззнаковое переполнение, то есть гигантский cap и
 * запись в соседнюю память. Здесь усечение просто останавливает добавление.
 */
static bool spotAppend(char *buf, size_t cap, size_t &o, const char *fmt, ...) {
  if (o >= cap) return false;
  va_list ap; va_start(ap, fmt);
  int w = vsnprintf(buf + o, cap - o, fmt, ap);
  va_end(ap);
  if (w < 0 || (size_t)w >= cap - o) { buf[cap - 1] = 0; o = cap; return false; }
  o += (size_t)w;
  return true;
}

static void spotHandleState() {
  spotTouch();
  SpotSnap s;
  if (!spotSnapRead(s)) { spotSendJson("{\"busy\":1}"); return; }

  size_t o = 0;
  spotAppend(spotJson, sizeof(spotJson), o,
    "{\"ms\":%lu,\"mode\":%u,\"modeName\":\"%s\",\"eng\":%d,\"pose\":%d,\"tab\":%d,"
    "\"net\":%u,\"ip\":\"%s\",\"hb\":%d,"
    "\"gi\":%u,\"phase\":%.3f,\"done\":%.2f,\"cyc\":%u,\"ramping\":%d,"
    "\"duty\":%.2f,\"amp\":%.1f,\"knee\":%.1f,\"per\":%.0f,\"hip\":%.1f,"
    "\"crouch\":%.0f,\"trim\":%.1f,\"link\":%.2f,\"rate\":%.0f,\"peak\":%.0f,"
    "\"turn\":%d,\"flat\":%d,\"smooth\":%d,"
    "\"hits\":%lu,\"over\":%lu,\"bus\":%lu,\"busmax\":%lu,\"heap\":%lu,\"j\":[",
    (unsigned long)s.ms, (unsigned)s.mode,
    s.mode < 5 ? SPOT_MODE_NAME[s.mode] : "?",
    s.engaged ? 1 : 0, s.havePose ? 1 : 0, s.tablesOk ? 1 : 0,
    (unsigned)spotNetState, spotNetIp, spotWebControls ? 1 : 0,
    (unsigned)s.gaitIdx, s.phase, s.cyclesDone, (unsigned)s.walkCycles,
    s.speedRamping ? 1 : 0,
    s.duty, s.ampThigh, s.ampKnee, s.cycleMs, s.ampHip,
    s.crouch, s.trim, s.linkRatio, s.maxRate, s.peakRate,
    (int)s.turn, s.flatPath ? 1 : 0, s.smoothSwing ? 1 : 0,
    (unsigned long)s.limitHits, (unsigned long)s.tickOverruns,
    (unsigned long)s.busUsLast, (unsigned long)s.busUsMax,
    (unsigned long)s.freeHeap);

  // Пределы берутся из spotty_config.h прямо здесь: они константы, гонять их
  // через снимок каждый тик незачем.
  bool room = true;
  for (uint8_t leg = 0; leg < LEG_COUNT && room; leg++) {
    for (uint8_t j = 0; j < JOINT_PER_LEG && room; j++) {
      uint8_t ch = SPOTTY_CH[leg][j];
      float lo = 0, hi = 0;
      spottyAngleLimits(ch, lo, hi);
      room = spotAppend(spotJson, sizeof(spotJson) - 4, o,
        "%s{\"ch\":%u,\"a\":%.1f,\"lo\":%.1f,\"hi\":%.1f}",
        (leg || j) ? "," : "", (unsigned)ch, s.cur[leg][j], lo, hi);
    }
  }
  spotAppend(spotJson, sizeof(spotJson), o, "]");

  // ШИМ, мкс по каналу в порядке "j". Тем же путём, что applyJoint:
  // spottyAngleToUs = кламп угла + кламп окна. Без ШИМ — 0 (серво обесточены).
  spotAppend(spotJson, sizeof(spotJson) - 4, o, ",\"us\":[");
  room = true;
  for (uint8_t leg = 0; leg < LEG_COUNT && room; leg++)
    for (uint8_t j = 0; j < JOINT_PER_LEG && room; j++) {
      uint8_t ch = SPOTTY_CH[leg][j];
      unsigned us = s.engaged ? (unsigned)spottyAngleToUs(ch, s.cur[leg][j]) : 0u;
      room = spotAppend(spotJson, sizeof(spotJson) - 4, o, "%s%u", (leg || j) ? "," : "", us);
    }
  spotAppend(spotJson, sizeof(spotJson) - 4, o, "]");

  spotAppend(spotJson, sizeof(spotJson) - 4, o,
    ",\"imu\":{\"ok\":%d,\"chip\":\"%s\",\"r\":%.2f,\"p\":%.2f,\"y\":%.2f,"
    "\"a\":[%.3f,%.3f,%.3f],\"stop\":%.0f}",
    s.imuOk ? 1 : 0, spImuName(), s.imuRoll, s.imuPitch, s.imuYaw,
    s.imuA[0], s.imuA[1], s.imuA[2], s.imuStop);

  spotAppend(spotJson, sizeof(spotJson) - 4, o,
    ",\"cart\":{\"eng\":%u,\"H\":%.1f,\"P\":%.1f,\"C\":%.1f,\"A\":%.1f,\"V\":%.1f,"
    "\"W\":%.0f,\"D\":%.1f,\"X\":%.1f,\"Y\":%.1f,\"R\":%.1f,\"B\":%.2f,"
    "\"pitch\":%.1f,\"yaw\":%.1f}",
    (unsigned)s.engine, s.cH, s.cP, s.cC, s.cA, s.cV, s.cW, s.cD, s.cX, s.cY,
    s.cR, s.cB, s.cPitch, s.cYaw);

  spotAppend(spotJson, sizeof(spotJson) - 4, o, ",\"trick\":%d,\"tk\":%u,\"thip\":%d",
    (int)s.trick, (unsigned)s.trickKey, s.trickHipOk ? 1 : 0);
  spotAppend(spotJson, sizeof(spotJson) - 4, o, ",\"prof\":%d", (int)s.prof);

  // Сеть: сила сигнала роутера (дБм, 0 в своей сети), своя ли сеть, сколько
  // устройств в ней, сколько раз переподключались. Читается здесь же, на ядре
  // 0, где живёт стек Wi-Fi.
  {
    const bool ap = spotNetState == NET_AP;
    spotAppend(spotJson, sizeof(spotJson) - 4, o, ",\"rssi\":%d,\"ap\":%d,\"cl\":%d,\"rc\":%lu",
      ap ? 0 : (int)WiFi.RSSI(), ap ? 1 : 0, ap ? (int)WiFi.softAPgetStationNum() : 0,
      (unsigned long)spotReconn);
  }

  spotAppend(spotJson, sizeof(spotJson), o, "}");
  spotSendJson(spotJson);
}

static void spotHandleLog() {
  spotTouch();
  uint32_t from = spotServer.hasArg("from")
                ? (uint32_t)strtoul(spotServer.arg("from").c_str(), nullptr, 10) : 0;

  portENTER_CRITICAL(&spotLogMux);
  uint32_t head = spotLogHead;
  uint32_t tail = (head > SPOTTY_LOG_CAP) ? (head - SPOTTY_LOG_CAP) : 0;
  bool     lost = from < tail;                 // клиент отстал, часть потеряна
  uint32_t pos  = lost ? tail : from;
  if (pos > head) pos = head;                  // перезапуск ESP32 при живой вкладке
  uint32_t n = head - pos;
  if (n > SPOTTY_LOG_CHUNK) n = SPOTTY_LOG_CHUNK;
  for (uint32_t i = 0; i < n; i++) spotRaw[i] = spotLogBuf[(pos + i) % SPOTTY_LOG_CAP];
  portEXIT_CRITICAL(&spotLogMux);

  n = spotTrimUtf8(spotRaw, n);
  size_t eaten = 0;
  spotJsonEscape(spotRaw, n, spotEsc, sizeof(spotEsc), &eaten);
  /*
   * Вывод переполнился и съел не всё. Подрезать одно только `eaten` нельзя:
   * в spotEsc уже лежит хвост оборванной последовательности, он уедет в JSON
   * битым И приедет второй раз следующим запросом (дубль в логе). Поэтому
   * подрезаем исходник по границе UTF-8 и экранируем ЗАНОВО.
   */
  if (eaten < n) {
    size_t fit = spotTrimUtf8(spotRaw, eaten);
    if (fit == 0) fit = eaten;            // не дать смещению застрять
    spotJsonEscape(spotRaw, fit, spotEsc, sizeof(spotEsc), &eaten);
    eaten = fit;
  }

  static char hdr[96];
  snprintf(hdr, sizeof(hdr), "{\"next\":%lu,\"head\":%lu,\"lost\":%d,\"txt\":\"",
           (unsigned long)(pos + eaten), (unsigned long)head, lost ? 1 : 0);
  spotServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  spotServer.sendHeader("Cache-Control", "no-store");
  spotServer.send(200, "application/json; charset=utf-8", "");
  spotServer.sendContent(hdr);
  spotServer.sendContent(spotEsc, strlen(spotEsc));
  spotServer.sendContent("\"}");
}

/*
 * ПОЧЕМУ КОМАНДА ТОЛЬКО POST И ТОЛЬКО SAME-ORIGIN.
 *
 * Пока это был GET, любая посторонняя страница, открытая в браузере
 * оператора, могла подать команду одной строкой <img src="http://spotty
 * .local/api/cmd?c=n">. Браузер не даст ей прочитать ответ, но ЗАПРОС УХОДИТ
 * И ИСПОЛНЯЕТСЯ. Команда 'n' — это первая подача ШИМ, та самая, перед которой
 * скетч пишет «серво рванут, руки убрать». Отсчёт не спасает: оператор о
 * команде не знает и в панель не смотрит.
 *
 * Модель угроз в шапке файла («кто в сети, тот и управляет») этот случай не
 * покрывала: атакующему не нужно быть в сети, достаточно чтобы оператор
 * открыл вкладку.
 *
 * Два барьера, оба бесплатные:
 *   - POST: <img>, <script> и ссылка так не умеют;
 *   - Sec-Fetch-Site: браузер проставляет его сам и подделать из страницы
 *     нельзя. Пустой заголовок пропускаем — это не браузер (curl, скрипт),
 *     а такой клиент и так уже в сети, то есть внутри принятой модели угроз.
 */
static void spotHandleCmd() {
  spotTouch();
  if (spotServer.method() != HTTP_POST) {
    spotServer.send(405, "text/plain", "only POST");
    return;
  }
  String site = spotServer.header("Sec-Fetch-Site");
  if (site.length() && site != "same-origin") {
    spotServer.send(403, "text/plain", "cross-site");
    return;
  }
  if (!spotCmdQ) { spotServer.send(503, "text/plain", "no queue"); return; }
  if (!spotServer.hasArg("c")) { spotServer.send(400, "text/plain", "no c"); return; }
  String c = spotServer.arg("c");
  c.trim();
  if (!c.length())                    { spotServer.send(400, "text/plain", "empty");  return; }
  if (c.length() >= SPOTTY_CMD_MAX)   { spotServer.send(400, "text/plain", "too long"); return; }

  SpotCmd cmd;
  strncpy(cmd.line, c.c_str(), SPOTTY_CMD_MAX - 1);
  cmd.line[SPOTTY_CMD_MAX - 1] = 0;

  if (xQueueSend(spotCmdQ, &cmd, 0) != pdTRUE) {
    spotServer.send(503, "text/plain", "queue full");
    return;
  }
  // Взводим watchdog: с этого момента пультом считается браузер.
  // 'x' и 'r' наоборот его снимают — оператор явно отпустил управление.
  if (cmd.line[0] == 'x' || cmd.line[0] == 'r') spotWebControls = false;
  else                                          spotWebControls = true;
  spotServer.send(200, "text/plain", "ok");
}

static void spotHandleNotFound() { spotServer.send(404, "text/plain", "404"); }

// ===========================================================================
// ЗАДАЧА ВЕБ-СЕРВЕРА — ЯДРО 0
// ===========================================================================
static void spotWebTask(void *) {
  // Незаполненный spotty_wifi.h — самая частая причина «не работает».
  // Двадцать секунд молчаливого ожидания здесь никому не помогают.
  const bool noCreds = !strcmp(SPOTTY_WIFI_SSID, "ЗАПОЛНИТЬ") || !SPOTTY_WIFI_SSID[0];
  WiFi.persistent(false);              // не трепать флеш записью настроек
  bool ap = (SPOTTY_WIFI_MODE == 1) || noCreds;

  if (!ap) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);              // сон радио = рваная задержка пульта
    WiFi.setAutoReconnect(true);
    WiFi.setHostname(SPOTTY_WIFI_HOST);
    WiFi.begin(SPOTTY_WIFI_SSID, SPOTTY_WIFI_PASS);
    spotNetState = NET_CONNECTING;
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < SPOTTY_WEB_CONN_MS)
      vTaskDelay(pdMS_TO_TICKS(200));
    if (WiFi.status() != WL_CONNECTED) {
      // Роутера нет. Раньше тут радио выключалось до перезагрузки, и пульта
      // не было весь сеанс. Теперь робот поднимает свою сеть.
      spotNetState = NET_FAILED;       // loop() напечатает причину
      vTaskDelay(pdMS_TO_TICKS(300));
      WiFi.disconnect(true);
      ap = true;
    }
  }
  if (ap) {
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    if (!WiFi.softAP(SPOTTY_AP_SSID, SPOTTY_AP_PASS, SPOTTY_AP_CHANNEL, 0, 2)) {
      spotNetState = NET_FAILED;       // и точка доступа не поднялась: только USB
      WiFi.mode(WIFI_OFF);
      vTaskDelete(nullptr);
      return;
    }
    strncpy(spotNetIp, WiFi.softAPIP().toString().c_str(), sizeof(spotNetIp) - 1);
  } else {
    strncpy(spotNetIp, WiFi.localIP().toString().c_str(), sizeof(spotNetIp) - 1);
  }
  spotNetIp[sizeof(spotNetIp) - 1] = 0;
  MDNS.begin(SPOTTY_WIFI_HOST);

  // Заголовки, которые WebServer иначе выбрасывает, не дочитывая.
  const char *want[] = { "Sec-Fetch-Site" };
  spotServer.collectHeaders(want, 1);

  spotServer.on("/",           HTTP_GET,  spotHandleRoot);
  spotServer.on("/api/state",  HTTP_GET,  spotHandleState);
  spotServer.on("/api/log",    HTTP_GET,  spotHandleLog);
  spotServer.on("/api/cmd",    HTTP_POST, spotHandleCmd);
  spotServer.onNotFound(spotHandleNotFound);
  spotServer.begin();
  __sync_synchronize();      // spotNetIp обязан быть виден раньше флага
  spotNetState = ap ? NET_AP : NET_UP;

  uint32_t lastTry = 0;
  for (;;) {
    spotServer.handleClient();
    if (!ap) {
      const bool up = WiFi.status() == WL_CONNECTED;
      if (!up && spotNetState == NET_UP) { spotNetState = NET_LOST; lastTry = millis(); }
      // Автопереподключение ядра иногда застревает (роутер перезагрузился,
      // сменил канал). Раз в 5 с толкаем сами.
      if (!up && spotNetState == NET_LOST && millis() - lastTry > SPOTTY_WEB_RETRY_MS) {
        lastTry = millis(); spotReconn++;
        WiFi.reconnect();
      }
      if (up && spotNetState == NET_LOST) {
        // DHCP мог выдать другой адрес; mDNS после обрыва надо объявить заново.
        strncpy(spotNetIp, WiFi.localIP().toString().c_str(), sizeof(spotNetIp) - 1);
        spotNetIp[sizeof(spotNetIp) - 1] = 0;
        MDNS.end(); MDNS.begin(SPOTTY_WIFI_HOST);
        __sync_synchronize();
        spotNetState = NET_UP;
      }
    }
    vTaskDelay(1);      // отдать ядро 0 стеку Wi-Fi
  }
}

// ===========================================================================
// ИНТЕРФЕЙС ДЛЯ СКЕТЧА — три вызова, все с ядра 1
// ===========================================================================

// Вызывать в конце setup(). Не блокирует: сеть поднимается в задаче.
inline void spottyWebBegin() {
  /*
   * Отказ выделения — не мелочь: xQueueSend(NULL,...) это configASSERT, то
   * есть ПЕРЕЗАГРУЗКА. А ребут при живом PCA9685 — ровно то, чем в шапке
   * обоснован отказ от OTA: драйвер держит последний ШИМ, потом отдаёт
   * непредсказуемое, робот под током падает. Поэтому без очереди веб просто
   * не поднимается, и скетч работает через USB.
   */
  spotCmdQ = xQueueCreate(SPOTTY_CMD_QUEUE, sizeof(SpotCmd));
  if (!spotCmdQ) {
    spotNetState = NET_FAILED;
    LOG.println("Веб НЕ поднят: не выделилась очередь команд. Работаем через USB.");
    return;
  }
  if (xTaskCreatePinnedToCore(spotWebTask, "spotweb", 8192, nullptr, 1, nullptr, 0)
      != pdPASS) {
    spotNetState = NET_FAILED;
    LOG.println("Веб НЕ поднят: не создалась задача сервера. Работаем через USB.");
  }
}

/*
 * Вызывать в начале loop(), рядом с pollSerial().
 * exec — функция скетча, исполняющая строку (обёртка над handleLine).
 *
 * За один вызов исполняется ОДНА команда, даже если в очереди больше. Так
 * тик не растягивается пачкой команд: очередь разберётся за несколько тиков,
 * то есть за единицы миллисекунд.
 */
inline void spottyWebPump(void (*exec)(char *)) {
  if (!spotCmdQ) return;
  SpotCmd cmd;
  if (xQueueReceive(spotCmdQ, &cmd, 0) == pdTRUE) exec(cmd.line);
}

/*
 * Watchdog связи. Вызывать в loop() каждый тик.
 * engagedNow — под током ли робот; stop — что делать при потере пульта.
 */
/*
 * atRisk — робот под током ЛИБО идёт отсчёт взвода. Второе обязательно:
 * во время MODE_ARM engaged ещё false, и если связь пропала сразу после
 * веб-команды 'n', отсчёт на ядре 1 спокойно доходит до конца и подаёт ШИМ
 * при отсутствующем пульте. Скетч передаёт сюда (engaged || mode==MODE_ARM),
 * а stop при взводе обязан взвод отменить.
 *
 * Печать нарочно в одну строку: она уходит в момент потери пульта, а на
 * 115200 каждые 100 байт это ~9 мс задержки тика.
 */
inline void spottyWebWatchdog(bool atRisk, void (*stop)()) {
  if (!spotWebControls || !atRisk) return;
  if ((millis() - spotLastContactMs) < SPOTTY_WEB_HB_MS) return;
  spotWebControls = false;
  LOG.printf("\n>> ПУЛЬТ ПОТЕРЯН (%lu мс тишины). ШИМ не снят.\n",
             (unsigned long)SPOTTY_WEB_HB_MS);
  stop();
}

// Печать смены состояния сети. Вызывать в loop(); печатает только по факту
// изменения, поэтому вызывать можно хоть каждый тик.
inline void spottyWebReport() {
  static uint8_t was = 255;
  uint8_t now = spotNetState;
  if (now == was) return;
  was = now;
  switch (now) {
    case NET_CONNECTING: LOG.printf("Wi-Fi: подключаюсь к \"%s\"...\n", SPOTTY_WIFI_SSID); break;
    case NET_UP:
      LOG.println();
      LOG.printf("Wi-Fi: подключено. ПАНЕЛЬ: http://%s/  либо http://%s.local/\n",
                 spotNetIp, SPOTTY_WIFI_HOST);
      LOG.println("Управление из браузера идёт через тот же разбор команд, что и USB.");
      LOG.println("USB остаётся рабочим одновременно с вебом.");
      break;
    case NET_FAILED:
      LOG.println();
      LOG.printf("Wi-Fi: подключиться к \"%s\" не удалось. Поднимаю свою сеть.\n", SPOTTY_WIFI_SSID);
      LOG.println("Проверь spotty_wifi.h и что сеть 2.4 ГГц: ESP32-S3 не видит 5 ГГц.");
      LOG.println("USB работает в любом случае.");
      break;
    case NET_AP:
      LOG.println();
      LOG.printf("Wi-Fi: СВОЯ СЕТЬ \"%s\", пароль \"%s\", канал %d.\n",
                 SPOTTY_AP_SSID, SPOTTY_AP_PASS, SPOTTY_AP_CHANNEL);
      LOG.printf("       Подключи ноутбук к ней. ПАНЕЛЬ: http://%s/  (адрес пульта: %s)\n",
                 spotNetIp, spotNetIp);
      break;
    case NET_LOST: LOG.println("Wi-Fi: связь потеряна, переподключаюсь каждые 5 с."); break;
    default: break;
  }
}
