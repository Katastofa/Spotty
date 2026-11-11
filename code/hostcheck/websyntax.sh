#!/bin/bash
# ===========================================================================
# Ворота компиляции ВЕБ-ЧАСТИ gait_test (SPOTTY_WEB 1). 2026-09-28.
#
# build.sh собирает скетч с SPOTTY_WEB 0 (desugar.sed), и spotty_web.h на
# стенде не компилировался вовсе. Здесь — -fsyntax-only того же файла с
# SPOTTY_WEB 1 на заглушках WiFi/WebServer/mDNS/FreeRTOS из webstub/.
# Линковки и исполнения нет: это ловит опечатки и типы, не поведение.
# ===========================================================================
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
B="${TMPDIR:-/tmp}/spotty_websyntax"
rm -rf "$B"; mkdir -p "$B"
cp "$HERE"/*.h "$B"/
cp -r "$HERE/webstub/." "$B"/
cp "$HERE/../gait_test/"*.h "$B"/
cp "$HERE/../spotty_config.h" "$B/parent_config.h"
cp "$HERE/../spotty_osc.h" "$B/spotty_osc.h"
cp "$HERE/../spotty_imu.h" "$B/spotty_imu.h"
cd "$B"
sed -i 's|#include "../spotty_config.h"|#include "parent_config.h"|' spotty_config.h
# Прототипы — как их вставляет Arduino IDE (иначе проскакивают ошибки вида
# "'TrickKey' does not name a type", которых g++ сам по себе не видит).
python3 "$HERE/arduino_proto.py" "$HERE/../gait_test/gait_test.ino" proto.ino || exit 1
printf '#include "Arduino.h"\n#include "Wire.h"\n#include "web_host.h"\n' | cat - proto.ino > sketch.cpp
grep -q '^#define SPOTTY_WEB 1' sketch.cpp || { echo "в скетче нет '#define SPOTTY_WEB 1'"; exit 1; }
if g++ -std=gnu++17 -fsyntax-only -DARDUINO=200 -I. sketch.cpp 2> err.txt; then
  echo "  [ OK ] gait_test с SPOTTY_WEB 1"
else
  echo "  [ПРОВАЛ] gait_test с SPOTTY_WEB 1"; sed -n '1,60p' err.txt; exit 1
fi
