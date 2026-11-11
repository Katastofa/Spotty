"""
Проверка профилей ходьбы (команда 'M'): каждый профиль из spotty_profiles.h
прогоняется через настоящую прошивку (стенд trace) — кламп по пределам,
отказ IK, пик скорости сустава против своего ограничителя m.

    python профили_проверка.py

Это проверка ПРЕДЕЛОВ железа, а не устойчивости. Угловой движок в PyBullet
полу не соответствует (там падает даже пресет, ходивший по полу 2026-08-29),
поэтому физика здесь не судья. Быстрые профили и «на весу» — на подвесе.
"""
import json, math, os, subprocess, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from spotty_sim import firmware

def main():
    B = firmware.build("trace")
    g = json.loads(subprocess.run([B, "geom=1"], capture_output=True, text=True).stdout)
    F, T = g["geom_mm"]["femur"], g["geom_mm"]["tibia"]
    csv = os.path.join(tempfile.gettempdir(), "spotty_prof.csv")
    bad = 0
    for i, p in enumerate(g["profiles"]):
        a = [B, "gait=%d" % p["gait"], "engine=%d" % p["engine"], "trim=%g" % p["b"], "maxrate=%g" % p["m"],
             "cycles=2", "settle=30", "out=" + csv]
        if p["engine"] == 0:
            a += ["amp=%g" % p["t"], "knee=%g" % p["k"], "duty=%g" % p["d"], "period=%g" % p["T"],
                  "crouch=%g" % p["q"], "flat=%d" % p["f"], "smooth=%d" % p["S"]]
        else:
            a += ["height=%g" % p["H"], "span=%g" % p["P"], "lift=%g" % p["C"], "vel=%g" % p["V"],
                  "swing=%g" % p["W"], "shiftgain=%g" % p["B"]]
        d = json.loads(subprocess.run(a, capture_output=True, text=True).stdout)
        row = open(csv, encoding="utf-8").readlines()[1].split(",")
        th, kn = float(row[2]), float(row[3])
        L = math.sqrt(F*F + T*T + 2*F*T*math.cos(math.radians(kn)))
        gg = math.acos((L*L + F*F - T*T) / (2*L*F))
        h = L * math.cos(math.radians(th) + gg)
        ok = (d["limit_hits"] == 0 and not d["ik_fail"] and not d["pose_ik_fail"]
              and d["peak_rate_dps"] <= d["max_rate_dps"] + 0.5)
        bad += not ok
        print("%2d %-8s %-4s высота %3.0f мм · период %5.0f мс · duty %.2f · пик %4.0f/%3.0f °/с · кламп %d%s" % (
            i, p["name"], "OK" if ok else "ПЛОХО", h, d["cycle_ms"], d["duty"], d["peak_rate_dps"],
            d["max_rate_dps"], d["limit_hits"], "  · НА ВЕСУ" if p["air"] else ""))
    os.remove(csv)
    print("все профили в пределах" if not bad else "ЕСТЬ ПРОФИЛИ ЗА ПРЕДЕЛАМИ: %d" % bad)
    return 1 if bad else 0

if __name__ == "__main__":
    sys.exit(main())
