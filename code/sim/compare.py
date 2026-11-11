"""
Картинка «до и после»: умолчания прошивки против того, что нашло обучение.

    python compare.py --params results/best_trot_seed1.json
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E
from spotty_sim.player import read_csv
from spotty_sim import firmware

HERE = os.path.dirname(os.path.abspath(__file__))


def series_for(params, m, urdf, gait, cycles):
    summ, csv = firmware.trace(dict(E.DEFAULTS, **params), cycles=cycles, gait=gait)
    ts, rows, settle = read_csv(csv)
    E._SIM.clear()
    sim = E._sim(m, urdf)
    met = sim.run(ts, rows, settle, dict(E.DEFAULTS, **params)["height"],
                  tick_ms=summ["tick_ms"], record=True)
    os.remove(csv)
    return met, summ


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", required=True)
    ap.add_argument("--cycles", type=float, default=4.0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    with open(args.params, "r", encoding="utf-8") as f:
        blob = json.load(f)
    gait = blob.get("gait", 0)
    m, urdf, geom = E.prepare_urdf(HERE)

    base, base_s = series_for({}, m, urdf, gait, args.cycles)
    best, best_s = series_for(blob["params"], m, urdf, gait, args.cycles)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
    for met, name, c in ((base, "умолчания прошивки", "tab:red"),
                         (best, "после обучения", "tab:blue")):
        s = met["series"]
        ax[0].plot(s["t"], [v * 1000 for v in s["x"]], c, label=name)
        ax[1].plot(s["t"], [v * 1000 for v in s["z"]], c, label=name)
        ax[2].plot(s["t"], s["roll"], c, label=name)
    ax[0].set_ylabel("вперёд, мм")
    ax[1].set_ylabel("высота корпуса, мм")
    ax[2].set_ylabel("крен, °")
    for a in ax:
        a.set_xlabel("время, с")
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    fig.suptitle("Spotty, походка %s: %s" % (
        base_s["gait"],
        "устоял" if not best["fell"] else "упал"))
    fig.tight_layout()
    out = args.out or os.path.join(HERE, "results",
                                   "compare_%s.png" % base_s["gait"])
    fig.savefig(out, dpi=130)
    print("записано: %s" % out)
    print("умолчания: прошёл %+.0f мм, %s" % (base["x_m"] * 1000,
                                              "УПАЛ" if base["fell"] else "устоял"))
    print("обучение:  прошёл %+.0f мм, %s" % (best["x_m"] * 1000,
                                              "УПАЛ" if best["fell"] else "устоял"))


if __name__ == "__main__":
    main()
