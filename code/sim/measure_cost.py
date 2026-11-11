"""
ЦЕНА НЕЗНАНИЯ: во что обходится каждое НЕснятое измерение.

Логика опыта. Прошивка продолжает считать по тем числам, которые в ней
записаны, — она ведь не знает, что они неточны. А МОДЕЛЬ робота собирается по
другому числу, из того коридора, в котором настоящая величина может лежать.
Расхождение между «во что верит прошивка» и «как устроен робот» и есть цена
незнания. Ровно так это работает и на железе.

Один набор походки на всё: walk, duty 0.75, H 150, боковой сдвиг корпуса
±12.8 мм — то, что рекомендовано Протоколом №2. Меняется по одному числу за
раз.

    python measure_cost.py
    python measure_cost.py --cycles 8

Читать так: «разброс пути» — насколько разные ответы даёт модель, если
величина гуляет по своему коридору. Если при этом где-то «упал», а где-то нет,
то незнание меняет не точность ответа, а сам ответ.
"""
import argparse
import json
import os
import random
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E, model as M

HERE = os.path.dirname(os.path.abspath(__file__))

# Замер -> (что крутим в МОДЕЛИ, значения коридора, чем мерить)
SWEEPS = [
    ("M1  вынос хипа, мм", "hip_mm", [8.75, 10.75, 12.75]),
    ("M2  бедро, мм", "femur_mm", [105.31, 108.31, 111.31]),
    ("M3  голень, мм", "tibia_mm", [113.51, 116.51, 119.51]),
    ("M3  радиус стопы (износ), мм", "foot_r_mm", [14.0, 19.41, 21.0]),
    ("M4  продольная база, мм", "base_long_mm", [135.0, 180.0, 232.0]),
    ("M5  поперечная база, мм", "base_lat_mm", [73.0, 78.0, 83.0]),
    ("масса корпуса, кг", "body_mass_kg", [0.70, 0.98, 1.40]),
    ("ЦТ вперёд/назад, мм", "body_com_x_mm", [-30.0, 0.0, 30.0]),
    ("ЦТ влево/вправо, мм", "body_com_y_mm", [-15.0, 0.0, 15.0]),
    ("ЦТ вверх/вниз, мм", "body_com_z_mm", [-60.0, 0.0, 30.0]),
    ("момент серво, Н·м", "stall_torque_nm", [1.5, 3.92, 6.0]),
    ("скорость серво, °/с", "no_load_dps", [150.0, 300.0, 600.0]),
    ("жёсткость контура kp", "servo_kp", [12.0, 20.0, 30.0]),
    ("латентность, мс", "latency_ms", [0.0, 12.0, 30.0]),
    ("трение стопы", "foot_friction", [0.5, 0.9, 1.4]),
]

# M9 — ошибки нулей. Отдельно, потому что это не одно число, а двенадцать.
ZEROS = [
    ("M9  нули идеальны (эталон)", "none", 0.0),
    ("M9  все бёдра +5° (общая ошибка)", "thigh_common", 5.0),
    ("M9  одно колено +7°", "one_knee", 7.0),
    ("M9  случайные ±3° по всем 12", "random", 3.0),
    ("M9  случайные ±7° по всем 12 (заявленный остаток)", "random", 7.0),
]


def make_zero_err(kind, amp, seed=7):
    err = [0.0] * 12
    if kind == "thigh_common":
        for leg in range(4):
            err[3 * leg + 1] = amp
    elif kind == "one_knee":
        err[2] = amp                      # FR колено
    elif kind == "random":
        rng = random.Random(seed)
        err = [rng.uniform(-amp, amp) for _ in range(12)]
    return err


def run(params, gait, cycles, model_over=None, zero_err=None):
    m, urdf, geom = E.prepare_urdf(HERE, gait=gait)
    for k, v in (model_over or {}).items():
        setattr(m, k, v)
        if k == "foot_friction":
            m.ground_friction = v
    with open(urdf, "w", encoding="utf-8") as f:
        f.write(M.build_urdf(m, limits_deg=E._limits(geom)))
    E._SIM.clear()
    sim = E._sim(m, urdf)
    sim.zero_err = zero_err or [0.0] * 12
    return E.evaluate(params, m, urdf, cycles=cycles, gait=gait)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=float, default=6.0)
    ap.add_argument("--gait", type=int, default=3)
    ap.add_argument("--height", type=float, default=150.0)
    ap.add_argument("--out", default=os.path.join(HERE, "results", "measure_cost.json"))
    args = ap.parse_args()

    m0, urdf0, geom = E.prepare_urdf(HERE, gait=args.gait)
    px, py = E.analytic_pattern(m0, geom["phase"], 0.75)
    base = {"height": args.height, "span": 25.0, "lift": 15.0, "vel": 40.0,
            "swing": E.swing_for_duty(25.0, 40.0, 0.75), "_px": px, "_py": py}
    params = E.decode("shift", [0, 0, 0.0, 1.0, 0], base)   # только боковой ±12.8

    print("НАБОР: walk, duty 0.75, H %.0f, боковой сдвиг корпуса ±12.8 мм, %d циклов"
          % (args.height, args.cycles))
    print("Прошивка во всех строках считает по СВОИМ числам; меняется РОБОТ.\n")
    print("  %-36s %22s %9s %7s" % ("замер", "коридор", "разброс", "вердикт"))
    print("  %-36s %22s %9s %7s" % ("", "путь, мм", "пути, мм", "меняется"))
    print("  " + "-" * 78)

    out = []
    for title, attr, values in SWEEPS:
        paths, fells, margins = [], [], []
        for v in values:
            score, met, summ, full = run(params, args.gait, args.cycles, {attr: v})
            paths.append(met["x_m"] * 1000)
            fells.append(met["fell"])
            margins.append(met["margin_mean_mm"])
        spread = max(paths) - min(paths)
        flips = "ДА" if len(set(fells)) > 1 else ("все упали" if all(fells) else "нет")
        print("  %-36s %22s %9.0f %7s"
              % (title, " / ".join("%+.0f" % p for p in paths), spread, flips))
        out.append({"what": title, "attr": attr, "values": values,
                    "paths": paths, "fell": fells, "margins": margins,
                    "spread": spread, "verdict_flips": flips})

    print("\n  M9 — ошибки нулей суставов (двенадцать чисел, не одно):")
    print("  %-52s %10s %8s %7s" % ("", "путь, мм", "запас", "упал"))
    for title, kind, amp in ZEROS:
        err = make_zero_err(kind, amp)
        score, met, summ, full = run(params, args.gait, args.cycles, None, err)
        print("  %-52s %+10.0f %8.1f %7s"
              % (title, met["x_m"] * 1000, met["margin_mean_mm"],
                 "ДА" if met["fell"] else "нет"))
        out.append({"what": title, "attr": "zero_err", "amp": amp, "kind": kind,
                    "paths": [met["x_m"] * 1000], "fell": [met["fell"]],
                    "margins": [met["margin_mean_mm"]]})

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print("\nзаписано: %s" % args.out)


if __name__ == "__main__":
    main()
