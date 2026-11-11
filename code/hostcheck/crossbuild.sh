#!/bin/bash
# ===========================================================================
# Кросс-сборка стенда под Windows. Нужна ровно для одного: чтобы ползунки в
# GUI симулятора работали на машине, где нет bash и g++ — там нельзя собрать
# прошивку на месте, а траекторию надо пересчитывать на каждое движение
# ползунка.
#
# Собирается ТОТ ЖЕ gait_test.ino теми же правилами, что в build.sh. Копии
# кода не появляется: меняется только компилятор.
#
#   ./crossbuild.sh trace ../sim/bin      -> ../sim/bin/trace.exe
#
# Требует mingw-w64 (apt install mingw-w64).
# ===========================================================================
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
WHAT="${1:-trace}"
DEST="${2:-$HERE/../sim/bin}"
CXX="${CXX_WIN:-x86_64-w64-mingw32-g++}"
command -v "$CXX" >/dev/null || { echo "нет $CXX (apt install mingw-w64)"; exit 1; }

B="${TMPDIR:-/tmp}/spotty_crossbuild"
rm -rf "$B"; mkdir -p "$B" "$DEST"
DEST="$(cd "$DEST" && pwd)"          # дальше будет cd, путь должен быть полным
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
sed -f "$HERE/desugar.sed" "$HERE/../gait_test/gait_test.ino" > body.cpp
printf '#include "Arduino.h"\n#include "Wire.h"\n' | cat - body.cpp > sketch.cpp
"$CXX" -std=c++11 -O2 -static -DARDUINO=200 -I. -o "$DEST/$WHAT.exe" \
       "$WHAT.cpp" sketch.cpp -lm
echo "собрано: $DEST/$WHAT.exe"
