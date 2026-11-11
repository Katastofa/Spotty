#!/bin/bash
# ===========================================================================
# Ворота компиляции для скетчей, которые нельзя прогнать без железа.
#
# build.sh собирает gait_test и ГОНЯЕТ по нему регрессию. Здесь другое и
# более скромное: servo_hold и servo_calib без робота исполнять нечего, но
# ОТ ОПЕЧАТКИ ОНИ ЗАЩИЩЕНЫ БЫТЬ ОБЯЗАНЫ. Правка, которую не пробовали даже
# компилировать, — это не правка, а предложение.
#
#   ./syntax.sh                 проверить все скетчи
#   ./syntax.sh servo_hold      проверить один
#   ./syntax.sh servo_calib run ещё и ВЫПОЛНИТЬ setup(): видно ровно то, что
#                               скетч напечатает при старте, без железа
#
# ВНИМАНИЕ про 'run': заглушка Wire отвечает ЗА ЛЮБОЙ адрес, поэтому скан I2C
# на стенде печатает все 127 адресов. Это свойство заглушки, а не шины.
#
# Проверяется РОВНО ТОТ файл, который заливается в ESP32-S3. Заглушки
# подменяют только железо: Arduino.h, Wire.h, Adafruit_PWMServoDriver.h,
# Preferences.h. Линковки нет — у скетча нет main(), и не должно быть.
# ===========================================================================
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
B="${TMPDIR:-/tmp}/spotty_syntax"
LIST="${1:-servo_hold servo_calib}"
RUN="${2:-}"
fail=0

for SK in $LIST; do
  SRC="$HERE/../$SK/$SK.ino"
  [ -f "$SRC" ] || { echo "нет файла: $SRC"; fail=1; continue; }
  rm -rf "$B"; mkdir -p "$B"
  cp "$HERE"/*.h "$B"/
  cp "$HERE/../spotty_config.h" "$B/parent_config.h"
  cp "$HERE/../spotty_osc.h" "$B/spotty_osc.h"   # 2026-09-26
  # Шим из папки скетча подключает конфиг по "../" — на стенде путь другой.
  printf '#pragma once\n#include "parent_config.h"\n' > "$B/spotty_config.h"
  cp "$SRC" "$B/sketch.cpp"
  printf '#include "Arduino.h"\n#include "Wire.h"\n' | cat - "$B/sketch.cpp" > "$B/s.cpp"
  if [ "$RUN" = "run" ]; then
    # Стенд вызывает setup() и ничего больше: цикла и ввода здесь нет.
    printf 'bool HOST_QUIET=false;\nHostSerial Serial;\nTwoWire Wire;\nvoid setup();\nint main(){ setup(); return 0; }\n' > "$B/run.cpp"
    printf '#include "Arduino.h"\n#include "Wire.h"\n' | cat - "$B/run.cpp" > "$B/r.cpp"
    if g++ -std=c++11 -DARDUINO=200 -I"$B" -o "$B/$SK.bin" "$B/r.cpp" "$B/s.cpp" 2> "$B/err.txt"; then
      echo "  [ OK ] $SK — вывод setup():"
      "$B/$SK.bin" || true
      continue
    fi
    echo "  [ПРОВАЛ] $SK"; sed -n '1,40p' "$B/err.txt"; fail=1; continue
  fi
  if g++ -std=c++11 -fsyntax-only -DARDUINO=200 -I"$B" "$B/s.cpp" 2> "$B/err.txt"; then
    echo "  [ OK ] $SK"
  else
    echo "  [ПРОВАЛ] $SK"
    sed -n '1,40p' "$B/err.txt"
    fail=1
  fi
done

[ $fail -eq 0 ] && echo "компиляция скетчей пройдена" || echo "ЕСТЬ ПРОВАЛЫ"
exit $fail
