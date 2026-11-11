#!/bin/bash
# ===========================================================================
# Стенд офлайн-проверки. Собирает НАСТОЯЩИЙ gait_test.ino обычным g++ и
# гоняет по нему регрессию. Копии кода здесь нет: проверяется ровно тот файл,
# который заливается в ESP32-S3. Заглушки Arduino.h / Wire.h /
# Adafruit_PWMServoDriver.h подменяют только железо — время, шину и драйвер
# ШИМ; вся арифметика настоящая.
#
#   ./build.sh            собрать и прогнать ворота регрессии
#   ./build.sh preview    что напечатает 'p' на роботе, без подачи питания
#   ./build.sh tune       перебор параметров под лимит по скорости сустава
#
# Сборка идёт в /tmp: в папку проекта не пишется ни одного файла.
# ===========================================================================
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
B="${TMPDIR:-/tmp}/spotty_hostcheck"
rm -rf "$B"; mkdir -p "$B"
cp "$HERE"/*.h "$HERE"/*.cpp "$B"/
cp "$HERE/../gait_test/oq_gait.h" "$B"/
cp "$HERE/../gait_test/spotty_tricks.h" "$B"/    # 2026-09-28, таблица трюков
cp "$HERE/../gait_test/spotty_tricks_types.h" "$B"/
cp "$HERE/../gait_test/spotty_profiles.h" "$B"/    # 2026-09-30, профили ходьбы
cp "$HERE/../gait_test/spotty_config.h" "$B"/
cp "$HERE/../spotty_config.h" "$B/parent_config.h"
cp "$HERE/../spotty_osc.h" "$B/spotty_osc.h"     # 2026-09-26, подключает "spotty_config.h" рядом
cp "$HERE/../spotty_imu.h" "$B/spotty_imu.h"     # 2026-09-26
cd "$B"
sed -i 's|#include "../spotty_config.h"|#include "parent_config.h"|' spotty_config.h
# Правила подготовки копии — в desugar.sed, ОДНИ на build.sh и crossbuild.sh.
# Там же объяснено, почему с BASE_LONG_MM снимается ещё и const.
sed -f "$HERE/desugar.sed" "$HERE/../gait_test/gait_test.ino" > body.cpp
printf '#include "Arduino.h"\n#include "Wire.h"\n' | cat - body.cpp > sketch.cpp
WHAT="${1:-harness}"
g++ -std=c++11 -O2 -DARDUINO=200 -I. -o "$WHAT.bin" "$WHAT.cpp" sketch.cpp -lm
# NORUN=1 — только собрать и напечатать путь к бинарю. Нужно симулятору:
# он зовёт бинарь сотни раз и пересобирать его каждый раз незачем.
if [ -n "$NORUN" ]; then echo "$B/$WHAT.bin"; exit 0; fi
shift || true
exec "./$WHAT.bin" "$@"
