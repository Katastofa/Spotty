"""
Повторяемость: один прогон — не результат.

Каждый набор гоняется по сетке ПРИНЯТЫХ чисел (трение, масса корпуса, ЦТ) и
считается, в скольких случаях робот устоял. Набор, который стоит только при
одном сочетании выдуманных величин, на железо не годится, как бы далеко он
ни уехал в лучшем прогоне.

    python robust.py            сравнение сдвигов корпуса на walk
"""
import argparse
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E, model as M

HERE = os.path.dirname(os.path.abspath(__file__))

# Сетка ПРИНЯТЫХ чисел, по которой проверяется набор.
GRID = [
    {},
    {"foot_friction": 0.7, "ground_friction": 0.7},
    {"foot_friction": 1.2, "ground_friction": 1.2},
    {"body_mass_kg": 0.85},
    {"body_mass_kg": 1.15},
    {"body_com_z_mm": -30.0},
    {"servo_kp": 16.0},
    {"latency_ms": 20.0},
]


def run_grid(params, gait, cycles, height):
    ok, dist, margins = 0, [], []
    for over in GRID:
        m, urdf, geom = E.prepare_urdf(HERE, gait=gait)
        for k, v in over.items():
            setattr(m, k, v)
        with open(urdf, "w", encoding="utf-8") as f:
            f.write(M.build_urdf(m, limits_deg=E._limits(geom)))
        E._SIM.clear()
        score, met, summ, full = E.evaluate(params, m, urdf, cycles=cycles, gait=gait)
        if not met["fell"]:
            ok += 1
        dist.append(met["x_m"] * 1000)
        margins.append(met["margin_mean_mm"])
    return ok, dist, margins


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, default=150.0)
    ap.add_argument("--cycles", type=float, default=6.0)
    ap.add_argument("--gait", type=int, default=3)
    ap.add_argument("--out", default=os.path.join(HERE, "results", "robust.json"))
    args = ap.parse_args()

    m, urdf, geom = E.prepare_urdf(HERE, gait=args.gait)
    px, py = E.analytic_pattern(m, geom["phase"], 0.75)
    base = {"height": args.height, "span": 25.0, "lift": 15.0, "vel": 40.0,
            "swing": E.swing_for_duty(25.0, 40.0, 0.75), "_px": px, "_py": py}

    # Подписи считаются от ФАКТИЧЕСКОЙ амплитуды, а не вписаны руками: она
    # зависит от продольной базы, и вписанное число соврало бы в тот же день,
    # когда базу перемеряют. amp — то, что требует геометрия, доля — гейн.
    amp = abs(py[0])
    cases = [
        ("без сдвига", [0, 0, 0.0, 0.0, 0]),
        ("вбок ±%.1f мм (пол-геометрии)" % (0.5 * amp), [0, 0, 0.0, 0.5, 0]),
        ("вбок ±%.1f мм (ГЕОМЕТРИЯ)" % amp, [0, 0, 0.0, 1.0, 0]),
        ("вбок ±%.1f мм (полторы)" % (1.5 * amp), [0, 0, 0.0, 1.5, 0]),
        ("вбок ±%.1f + корпус вперёд 20" % (0.5 * amp), [20, 0, 0.0, 0.5, 0]),
        ("вбок ±%.1f + корпус вперёд 20" % amp, [20, 0, 0.0, 1.0, 0]),
        ("геометрия целиком (и вперёд, и вбок)", [0, 0, 1.0, 1.0, 0]),
    ]
    # Наборы выше строит СТЕНД: он подменяет pShiftYMm перед каждым вызовом.
    # Набор ниже строит ПРОШИВКА: 'B 1' включает её собственную таблицу.
    # Строка «ГЕОМЕТРИЯ» и «таблица прошивки» обязаны совпасть — таблицы у них
    # получены независимо (питон через инцентр, прошивка через sqrt(a²+b²)−a
    # и фазировку), и расхождение означало бы ошибку в одной из них. Разница в
    # числах ниже — это не расхождение таблиц, а чувствительность самой сетки:
    # 3·10⁻⁵ градуса на входе уже переворачивают один прогон из восьми.
    fw = {k: v for k, v in base.items() if not k.startswith("_")}
    fw["shiftgain"] = 1.0
    print("walk, H %.0f, duty 0.75, %d прогонов на набор по сетке ПРИНЯТЫХ чисел"
          % (args.height, len(GRID)))
    print("продольная база %.0f мм — %s; геометрия требует сдвига ±%.2f мм"
          % (m.base_long_mm,
             "ИЗМЕРЕНО (M4)" if m.base_long_measured else "ПРИНЯТО, M4 не снят",
             amp))
    print("  %-38s %8s %10s %10s" % ("набор", "устоял", "путь медиана", "запас сред"))
    out = []
    for name, vec in cases + [("ТАБЛИЦА ПРОШИВКИ ('B 1')", None)]:
        params = fw if vec is None else E.decode("shift", vec, base)
        ok, dist, margins = run_grid(params, args.gait, args.cycles, args.height)
        print("  %-38s %4d/%-3d %+10.0f %10.1f"
              % (name, ok, len(GRID), statistics.median(dist),
                 statistics.median(margins)))
        out.append({"name": name, "vec": vec, "ok": ok, "n": len(GRID),
                    "dist": dist, "margins": margins, "params": params})
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print("записано: %s" % args.out)


if __name__ == "__main__":
    main()
