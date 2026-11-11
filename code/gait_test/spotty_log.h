#pragma once
/*
 * spotty_log.h — Serial + копия в кольцевой буфер для веб-панели.
 *
 * ПОЧЕМУ ОТДЕЛЬНЫМ ФАЙЛОМ, а не внутри spotty_web.h. Порядок включения:
 * spotty_config.h тоже печатает (самопроверка пределов), и его печать должна
 * попадать в панель — иначе оператор увидит «ТАБЛИЦЫ НЕ ПРОШЛИ» в мониторе
 * порта и не увидит в браузере. Значит LOG обязан существовать РАНЬШЕ
 * конфига, а spotty_web.h требует типов ИЗ конфига. Разрываем кольцо:
 *
 *     spotty_log.h  ->  spotty_config.h  ->  spotty_web.h
 *
 * ОДИН ПИСАТЕЛЬ. В кольцо пишет только ядро 1 (loop и setup). Веб-задача на
 * ядре 0 только читает и в лог не пишет никогда — иначе понадобился бы мьютекс
 * на пути печати, то есть внутри тика 3 мс.
 */

#include <Arduino.h>

static const size_t SPOTTY_LOG_CAP = 8192;      // кольцо, байт

static char              spotLogBuf[SPOTTY_LOG_CAP];
static volatile uint32_t spotLogHead = 0;       // всего байт за всё время
static portMUX_TYPE      spotLogMux  = portMUX_INITIALIZER_UNLOCKED;

static void spotLogWrite(const uint8_t *data, size_t len) {
  if (!len) return;
  if (len > SPOTTY_LOG_CAP) {            // строка длиннее кольца — хвост важнее
    data += (len - SPOTTY_LOG_CAP);
    len = SPOTTY_LOG_CAP;
  }
  portENTER_CRITICAL(&spotLogMux);
  uint32_t h = spotLogHead;
  for (size_t i = 0; i < len; i++) spotLogBuf[(h + i) % SPOTTY_LOG_CAP] = (char)data[i];
  spotLogHead = h + len;
  portEXIT_CRITICAL(&spotLogMux);
}

/*
 * LOG — то же, что Serial, плюс копия в кольцо.
 *
 * Почему подменён Serial, а не добавлена вторая печать рядом: печатей в
 * скетче под две сотни, и «забыть продублировать одну» — это ровно тот класс
 * ошибок, из-за которого оператор не увидит в панели предупреждение, видное
 * в мониторе порта. Замена механическая: Serial.print* -> LOG.print*,
 * а Serial.begin/available/read остались Serial.
 *
 * Print::printf и Print::println собирают строку и отдают её ОДНИМ вызовом
 * write(buf, len), поэтому критическая секция берётся раз на строку, а не
 * раз на символ.
 */
class SpotLogPrint : public Print {
public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *buf, size_t n) override {
    Serial.write(buf, n);
    spotLogWrite(buf, n);
    return n;
  }
};

SpotLogPrint LOG;
