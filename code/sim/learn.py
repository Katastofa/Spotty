"""
Запуск обучения.

    python learn.py                            рысь, 20 поколений по 12 вариантов
    python learn.py --gait 3 --gens 25         походка walk, дольше
    python learn.py --seed 7 --workers 4
    python learn.py --base-long 135            проверить, как выводы зависят от M4

Результат кладётся в results/: JSON с историей и лучшими параметрами, PNG со
сходимостью и CSV лучшей траектории для просмотра в GUI.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E, optimize as O

HERE = os.path.dirname(os.path.abspath(__file__))
GAIT_NAMES = {0: "trot", 1: "pace", 2: "bound", 3: "walk", 4: "gallop"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gait", type=int, default=0)
    ap.add_argument("--gens", type=int, default=20)
    ap.add_argument("--lam", type=int, default=12)
    ap.add_argument("--cycles", type=float, default=4.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--base-long", type=float, default=None)
    ap.add_argument("--from-defaults", action="store_true",
                    help="начать поиск от умолчаний прошивки, а не от центра коридора")
    ap.add_argument("--out", default=os.path.join(HERE, "results"))
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    tag = "%s_seed%d" % (GAIT_NAMES.get(args.gait, str(args.gait)), args.seed)
    log = os.path.join(args.out, "learn_%s.json" % tag)

    m, urdf, geom = E.prepare_urdf(HERE, base_long_mm=args.base_long)
    print("=" * 72)
    print("СТАТУС ЧИСЕЛ МОДЕЛИ (SIMULATION.md R3)")
    for what, status, why in m.status_table():
        print("  %-14s %s — %s" % (status, what, why))
    print("=" * 72)
    print("походка %s, фазы %s" % (geom["gait"], geom["phase"]))
    print("поиск по: " + ", ".join("%s %g…%g" % (k, v[0], v[1])
                                   for k, v in sorted(E.SPACE.items())))
    print("=" * 72)

    base, base_met, base_summ, base_full = E.evaluate(
        {}, m, urdf, cycles=args.cycles, gait=args.gait)
    print("отправная точка (умолчания прошивки): фитнес %+.4f, прошёл %+.0f мм, %s"
          % (base, base_met["x_m"] * 1000, "УПАЛ" if base_met["fell"] else "устоял"))

    best, hist = O.cem(gens=args.gens, lam=args.lam, seed=args.seed, gait=args.gait,
                       cycles=args.cycles, base_long=args.base_long,
                       workers=args.workers, log_path=log,
                       start=E.DEFAULTS if args.from_defaults else None)

    score, params, met, summ = best
    print("=" * 72)
    print("ЛУЧШЕЕ: фитнес %+.4f (было %+.4f)" % (score, base))
    print("  " + "  ".join("%s=%.1f" % (k, v) for k, v in sorted(params.items())))
    print("  период %.0f мс, duty %.2f, пик по суставу %.0f °/с из %.0f разрешённых"
          % (summ["cycle_ms"], summ["duty"], summ["peak_rate_dps"], summ["max_rate_dps"]))
    print("  прошёл %+.0f мм за %.1f с (%.0f мм/с), снос %+.0f мм, курс %+.1f°"
          % (met["x_m"] * 1000, met["walk_s"], met["speed_mm_s"],
             met["y_m"] * 1000, met["yaw_deg"]))
    print("  качка СКЗ крен %.2f° тангаж %.2f°, проскальзывание %.0f мм, %s"
          % (met["roll_rms_deg"], met["pitch_rms_deg"], met["slip_mm"],
             "УПАЛ" if met["fell"] else "устоял"))
    print("  обрезка по жёсткому пределу: %d раз%s"
          % (summ.get("limit_hits", 0),
             "" if not summ.get("limit_hits") else "  — цикл НЕ готов к железу"))

    with open(os.path.join(args.out, "best_%s.json" % tag), "w", encoding="utf-8") as f:
        json.dump({"params": params, "metrics": met, "firmware": summ,
                   "baseline": {"params": base_full, "score": base,
                                "metrics": base_met},
                   "score": score, "gait": args.gait, "seed": args.seed},
                  f, ensure_ascii=False, indent=1)

    # траектория лучшего — чтобы посмотреть её глазами в GUI
    csv = os.path.join(args.out, "traj_best_%s.csv" % tag)
    E.evaluate(params, m, urdf, cycles=args.cycles, gait=args.gait, keep_csv=csv)
    print("  траектория: %s" % csv)
    print("  журнал:     %s" % log)
    _plot(hist, os.path.join(args.out, "learn_%s.png" % tag), tag, base)


def _plot(hist, path, tag, baseline):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    g = [h["gen"] for h in hist]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    ax[0].plot(g, [h["best"] for h in hist], label="лучший в поколении")
    ax[0].plot(g, [h["mean_elite"] for h in hist], label="среднее по элите")
    ax[0].plot(g, [h["median"] for h in hist], label="медиана", alpha=0.5)
    ax[0].axhline(baseline, ls="--", c="k", lw=1, label="умолчания прошивки")
    ax[0].set_xlabel("поколение")
    ax[0].set_ylabel("фитнес")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    keys = sorted(hist[0]["mu"])
    for k in keys:
        lo, hi = __import__("spotty_sim.evaluate", fromlist=["E"]).SPACE[k]
        ax[1].plot(g, [(h["mu"][k] - lo) / (hi - lo) for h in hist], label=k)
    ax[1].set_xlabel("поколение")
    ax[1].set_ylabel("центр облака, доля от диапазона")
    ax[1].set_ylim(0, 1)
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    fig.suptitle("Обучение походки Spotty — %s" % tag)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print("  график:     %s" % path)


if __name__ == "__main__":
    main()
