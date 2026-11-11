"""
Один прогон с заданными параметрами. Без обучения — просто «что будет, если».

    python run_once.py                       умолчания прошивки
    python run_once.py span=40 vel=90        свои параметры
    python run_once.py --gui span=40         с окном, в реальном времени
    python run_once.py --params best.json    из файла, который оставило обучение
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E


def parse(argv):
    gui = "--gui" in argv
    argv = [a for a in argv if a != "--gui"]
    params, cycles, gait = {}, 4.0, 0
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--params":
            i += 1
            # encoding задан ЯВНО: в файлах результатов есть русский текст,
            # а на русской Windows open() по умолчанию читает cp1251 и падает.
            # Второй open() тут же был лишним — файл уже прочитан.
            with open(argv[i], "r", encoding="utf-8") as f:
                blob = json.load(f)
            params.update(blob.get("params", blob))
        elif "=" in a:
            k, v = a.split("=", 1)
            if k == "cycles":
                cycles = float(v)
            elif k == "gait":
                gait = int(v)
            else:
                params[k] = float(v)
        i += 1
    return gui, params, cycles, gait


def main():
    gui, params, cycles, gait = parse(sys.argv[1:])
    m, urdf, geom = E.prepare_urdf(os.path.dirname(os.path.abspath(__file__)))
    print("продольная база %.0f мм — %s" % (
        m.base_long_mm,
        "ИЗМЕРЕНО (M4, рулетка)" if m.base_long_measured
        else "ПРИНЯТО, M4 не снят"))
    score, met, summ, full = E.evaluate(params, m, urdf, cycles=cycles, gait=gait,
                                        gui=gui, realtime=gui)
    print("походка %s, период %.0f мс, duty %.2f, пик по суставу %.0f °/с (предел %.0f)"
          % (summ["gait"], summ["cycle_ms"], summ["duty"],
             summ["peak_rate_dps"], summ["max_rate_dps"]))
    print("параметры: " + "  ".join("%s=%.1f" % kv for kv in sorted(full.items())))
    print("-" * 64)
    print("прошёл вперёд      %+7.1f мм за %.1f с  (%.0f мм/с, заказано %.0f)"
          % (met["x_m"] * 1000, met["walk_s"], met["speed_mm_s"], full["vel"]))
    print("снос вбок          %+7.1f мм     курс %+.1f°" % (met["y_m"] * 1000, met["yaw_deg"]))
    print("качка СКЗ          крен %.2f°  тангаж %.2f°" % (met["roll_rms_deg"], met["pitch_rms_deg"]))
    print("проскальзывание    %.0f мм" % met["slip_mm"])
    print("работа приводов    %.1f Дж     отставание сустава %.2f°"
          % (met["energy_j"], met["track_deg"]))
    print("упал               %s" % ("ДА" if met["fell"] else "нет"))
    print("-" * 64)
    print("фитнес %.4f" % score)
    if gui:
        input("Enter — закрыть окно ")


if __name__ == "__main__":
    main()
