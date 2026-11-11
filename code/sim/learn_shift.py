"""
ФАЗОВЫЙ СДВИГ КОРПУСА — то, ради чего симулятор и затевался (§0 SIMULATION.md).

Постановка опыта. Всё, кроме сдвига, ЗАФИКСИРОВАНО: походка, высота, шаг,
подъём, скорость, duty. Сам сдвиг задан таблицей из восьми узлов на цикл, но
ищутся не узлы вслепую: нужный сдвиг СЧИТАЕТСЯ из геометрии опорного
треугольника (инцентр), а подбираются пять чисел — во сколько раз взять его
вперёд, во сколько вбок, с каким опережением по фазе и с каким постоянным
перекосом. Каждое из пяти можно обсуждать словами, в отличие от шестнадцати
безымянных узлов.

Прошивка при этом НЕ МЕНЯЕТСЯ. Стенд ставит её же `pShiftXMm` / `pShiftYMm`
перед каждым вызовом `gaitPose`, то есть проверяет, стоит ли вообще заводить
в прошивке таблицу сдвига и какой ей быть.

Целевая метрика — ЗАПАС УСТОЙЧИВОСТИ: знаковое расстояние от проекции центра
масс до границы опорного многоугольника. У четвероногого, стоящего на трёх
ногах с ЦТ ровно в середине рамы, этот запас РАВЕН НУЛЮ по построению:
диагональ опорного треугольника проходит через центр. Сдвиг корпуса — это и
есть способ сделать его положительным.

    python learn_shift.py                       H 190 (умолчание прошивки)
    python learn_shift.py --height 150
    python learn_shift.py --gens 30 --lam 20
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E, optimize as O

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gait", type=int, default=3, help="3 = walk")
    ap.add_argument("--height", type=float, default=190.0)
    ap.add_argument("--span", type=float, default=25.0)
    ap.add_argument("--lift", type=float, default=15.0)
    ap.add_argument("--vel", type=float, default=40.0)
    ap.add_argument("--duty", type=float, default=0.75)
    ap.add_argument("--gens", type=int, default=30)
    ap.add_argument("--lam", type=int, default=18)
    ap.add_argument("--cycles", type=float, default=4.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--out", default=os.path.join(HERE, "results"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    fixed = {
        "height": args.height, "span": args.span, "lift": args.lift,
        "vel": args.vel,
        "swing": E.swing_for_duty(args.span, args.vel, args.duty),
        "shiftx": 0.0, "shifty": 0.0,
    }
    w = E.Weights(w_margin=0.02, w_margin_neg=0.5)

    m, urdf, geom = E.prepare_urdf(HERE, gait=args.gait)
    px, py = E.analytic_pattern(m, geom["phase"], args.duty)
    fixed["_px"], fixed["_py"] = px, py
    tag = "shift_h%d_seed%d" % (int(args.height), args.seed)
    print("=" * 72)
    for what, status, why in m.status_table():
        print("  %-14s %s — %s" % (status, what, why))
    print("=" * 72)
    print("зафиксировано: " + "  ".join("%s=%.1f" % kv for kv in sorted(fixed.items())
                                         if not kv[0].startswith("_")))
    print("геометрия требует сдвига (инцентр опорного треугольника):")
    print("  вперёд, мм: " + " ".join("%+6.1f" % v for v in px))
    print("  вбок,   мм: " + " ".join("%+6.1f" % v for v in py))
    print("ищется: во сколько раз взять этот сдвиг, с каким опережением и перекосом")
    print("=" * 72)

    clean = {k: v for k, v in fixed.items() if not k.startswith("_")}
    base, bmet, bsumm, bfull = E.evaluate(clean, m, urdf, cycles=args.cycles,
                                          gait=args.gait, weights=w)
    print("БЕЗ СДВИГА: фитнес %+.4f | путь %+.0f мм | запас: мин %.1f сред %.1f мм | "
          "вне опоры %.0f%% времени | %s"
          % (base, bmet["x_m"] * 1000, bmet["margin_min_mm"], bmet["margin_mean_mm"],
             100 * bmet["margin_neg_frac"], "УПАЛ" if bmet["fell"] else "устоял"))
    print("период %.0f мс, duty %.2f" % (bsumm["cycle_ms"], bsumm["duty"]))

    geo = E.decode("shift", [0.0, 0.0, 1.0, 1.0, 0.0], fixed)   # bias 0, gain 1
    gsc, gmet, gsumm, _ = E.evaluate(geo, m, urdf, cycles=args.cycles,
                                     gait=args.gait, weights=w)
    print("ЧИСТО ГЕОМЕТРИЯ (gain 1, без подбора): фитнес %+.4f | путь %+.0f мм | "
          "запас: мин %.1f сред %.1f мм | вне опоры %.0f%% | %s"
          % (gsc, gmet["x_m"] * 1000, gmet["margin_min_mm"], gmet["margin_mean_mm"],
             100 * gmet["margin_neg_frac"], "УПАЛ" if gmet["fell"] else "устоял"))
    print("-" * 72)

    log = os.path.join(args.out, "learn_%s.json" % tag)
    best, hist = O.cem(gens=args.gens, lam=args.lam, seed=args.seed, gait=args.gait,
                       cycles=args.cycles, workers=args.workers, log_path=log,
                       weights=w, mode="shift", fixed=fixed)
    score, vec, met, summ = best
    print("-" * 72)
    print("СО СДВИГОМ: фитнес %+.4f | путь %+.0f мм | запас: мин %.1f сред %.1f мм | "
          "вне опоры %.0f%% времени | %s"
          % (score, met["x_m"] * 1000, met["margin_min_mm"], met["margin_mean_mm"],
             100 * met["margin_neg_frac"], "УПАЛ" if met["fell"] else "устоял"))
    params = E.decode("shift", [vec[k] for k in sorted(E.SHIFT_SPACE)], fixed)
    print("  сдвиг вперёд по фазе, мм: " + " ".join("%+6.1f" % v for v in params["sx"]))
    print("  сдвиг вбок   по фазе, мм: " + " ".join("%+6.1f" % v for v in params["sy"]))
    print("  обрезка по жёсткому пределу: %d раз" % summ.get("limit_hits", 0))

    out = {"params": params, "metrics": met, "firmware": summ, "score": score,
           "knobs": vec, "gait": args.gait, "seed": args.seed,
           "fixed": {k: v for k, v in fixed.items() if not k.startswith("_")},
           "pattern": {"px": px, "py": py},
           "baseline": {"score": base, "metrics": bmet, "firmware": bsumm},
           "geometry_only": {"score": gsc, "metrics": gmet, "params": geo}}
    with open(os.path.join(args.out, "best_%s.json" % tag), "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    csv = os.path.join(args.out, "traj_%s.csv" % tag)
    E.evaluate(params, m, urdf, cycles=args.cycles, gait=args.gait, keep_csv=csv,
               weights=w)
    print("  траектория: %s" % csv)
    print("  журнал:     %s" % log)


if __name__ == "__main__":
    main()
