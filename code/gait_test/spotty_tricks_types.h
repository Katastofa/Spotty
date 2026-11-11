#pragma once
// Типы таблицы трюков (команда 'e'). Отдельно от самой таблицы и от скетча:
// Arduino IDE генерирует прототипы функций .ino и ставит их ПЕРЕД первой
// функцией скетча, поэтому типы из сигнатур обязаны быть объявлены в
// заголовке, подключённом в начале. Иначе: "'TrickKey' does not name a type".
#include "spotty_config.h"

struct TrickKey { uint16_t ms; int16_t d[LEG_COUNT][3]; };   // мс; смещения стоп, 0.1 мм
struct TrickDef {
  const char *name, *ru, *note;
  const TrickKey *k; uint8_t n;
  bool endSit;      // кончается сидя (havePose = false), иначе — поза
  bool hip;         // двигает хипами: нужен проверенный знак
};
