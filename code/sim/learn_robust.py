"""
ОБУЧЕНИЕ ПОХОДКЕ, КОТОРАЯ НЕ ЗАВИСИТ ОТ НЕИЗМЕРЕННОГО.

Обычный поиск (learn.py) оценивает набор ОДНИМ прогоном при ОДНИХ ПРИНЯТЫХ
числах. Пока массы не взвешены, это самообман: оптимизатор честно найдёт
набор, блестящий при массе корпуса ровно 0.98 кг — числе, которое мы сами же
и выдумали. На железе этой массы не окажется, и походка развалится не потому,
что поиск плохой, а потому, что его спросили не о том.

Здесь спрашивают о другом: **какой набор прилично идёт при ЛЮБОЙ массе из
правдоподобного диапазона.** Каждый кандидат гоняется по сетке миров, и балл
сводится как

        среднее МИНУС половина разброса,

а не средним. Среднее охотно покупает дальний путь в одном мире ценой
падения в другом; среднее минус разброс — не покупает.

Что в сетке: масса корпуса ±20 %, массы звеньев ×0.7 и ×1.4, ЦТ корпуса ниже
на 30 мм. Это ровно те величины, под которые нужны весы. Трение в сетку НЕ
входит намеренно: оно меряется наклонной доской за десять минут, и гадать о
нём незачем — проще измерить.

    python learn_robust.py                       поиск при m=100 (как в прошивке)
    python learn_robust.py --maxrate 220         при снятом ограничителе
    python learn_robust.py --gens 12 --lam 20    дольше и шире

ПОСЛЕ ПОИСКА идёт проверка победителя на РАСШИРЕННОЙ сетке, в которую входят
и трение, и жёсткость контура, и латентность — то есть на числах, по которым
он НЕ обучался. Набор, который хорош только на своей сетке, здесь и сыпется.
"""
import argparse
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E, model as M, optimize as O

HERE = os.path.dirname(os.path.abspath(__file__))

# Сетка ОБУЧЕНИЯ: только то, для чего нужны весы.
GRID_LEARN = [
    {},
    {"body_mass_kg": 0.78},
    {"body_mass_kg": 1.18},
    {"hip_mass_kg": 0.105, "thigh_mass_kg": 0.126, "shank_mass_kg": 0.035},
    {"hip_mass_kg": 0.210, "thigh_mass_kg": 0.252, "shank_mass_kg": 0.070},
    {"body_com_z_mm": -30.0},
]

# Сетка ПРОВЕРКИ: обучающая плюс то, чего поиск не видел.
GRID_TEST = GRID_LEARN + [
    {"foot_friction": 0.7, "ground_friction": 0.7},
    {"foot_friction": 1.2, "ground_friction": 1.2},
    {"servo_kp": 14.0},
    {"servo_kp": 28.0},
    {"latency_ms": 20.0},
    {"body_com_x_mm": 20.0},
    {"stall_torque_nm": 2.94},
]


def check(params, grid, gait, cycles):
    """Прогнать готовый набор по сетке и вернуть сводку."""
    ok, dist, marg, trk, falls = 0, [], [], [], []
    for i, over in enumerate(grid):
        m, urdf, geom = E.prepare_urdf(HERE, gait=gait)
        for k, v in over.items():
            setattr(m, k, v)
        with open(urdf, "w", encoding="utf-8") as f:
            f.write(M.build_urdf(m, limits_deg=E._limits(geom)))
        E._SIM.clear()
        score, met, summ, full = E.evaluate(params, m, urdf, cycles=cycles, gait=gait)
        if met["fell"]:
            falls.append(", ".join("%s=%g" % kv for kv in over.items()) or "номинал")
        else:
            ok += 1
        dist.append(met["x_m"] * 1000)
        marg.append(met["margin_mean_mm"])
        trk.append(met["track_deg"])
    return {"ok": ok, "n": len(grid), "dist": dist, "margins": marg,
            "track": trk, "falls": falls, "summary": summ}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gait", type=int, default=3)        # 3 = walk
    ap.add_argument("--gens", type=int, default=10)
    ap.add_argument("--lam", type=int, default=16)
    ap.add_argument("--cycles", type=float, default=6.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--maxrate", type=float, default=100.0)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--gui", action="store_true",
                    help="показывать окно: видно, как шагает каждый кандидат")
    ap.add_argument("--realtime", action="store_true",
                    help="с --gui: проигрывать в реальном времени, а не бегом")
    args = ap.parse_args()
    if args.gui and args.workers != 1:
        # Не «исправляем молча»: окно и параллельность несовместимы, и
        # оператор должен понимать, за что платит.
        print("[i] С окном поиск идёт в один процесс: --workers %d -> 1."
              % args.workers)
        print("    Это примерно в %d раз(а) дольше. Без окна параллельность"
              " вернётся сама." % args.workers)
        args.workers = 1

    tag = args.tag or ("m%d" % args.maxrate)
    m0, _, _ = E.prepare_urdf(HERE, gait=args.gait)
    print("=" * 76)
    print("ОБУЧЕНИЕ, УСТОЙЧИВОЕ К НЕИЗМЕРЕННЫМ МАССАМ")
    print("=" * 76)
    for what, status, why in m0.status_table():
        print("  %-58s %-16s %s" % (what[:58], status, why[:40]))
    print()
    print("  походка %d, %d поколений x %d кандидатов x %d миров = %d прогонов"
          % (args.gait, args.gens, args.lam, len(GRID_LEARN),
             args.gens * args.lam * len(GRID_LEARN)))
    print("  ограничитель скорости сустава m = %.0f °/с" % args.maxrate)
    if args.gui:
        print("  ОКНО ВКЛЮЧЕНО: показывается номинальный мир, остальные пять")
        print("  считаются без картинки. Закрыть окно = прервать поиск.")
        if not args.realtime:
            print("  Проигрывание БЕГОМ. Реальное время: добавьте --realtime.")
    else:
        print("  окно выключено (быстрее). Показать ход поиска: --gui")
    print()

    log = os.path.join(HERE, "results", "learn_robust_%s.json" % tag)
    os.makedirs(os.path.dirname(log), exist_ok=True)
    best, hist = O.cem(gens=args.gens, lam=args.lam, seed=args.seed,
                       gait=args.gait, cycles=args.cycles,
                       workers=args.workers, mode="robust",
                       fixed={"maxrate": args.maxrate},
                       grid=GRID_LEARN, log_path=log,
                       gui=args.gui, realtime=args.realtime)

    params = dict(best[1])
    params["maxrate"] = args.maxrate
    print()
    print("-" * 76)
    print("ЛУЧШИЙ НАБОР")
    for k in sorted(params):
        print("    %-10s %8.2f" % (k, params[k]))

    print()
    print("ПРОВЕРКА на расширенной сетке (%d миров, часть которых поиск не видел)"
          % len(GRID_TEST))
    res = check(params, GRID_TEST, args.gait, args.cycles)
    print("  устоял %d из %d" % (res["ok"], res["n"]))
    print("  путь: медиана %+.0f мм, худший %+.0f мм"
          % (statistics.median(res["dist"]), min(res["dist"])))
    print("  запас устойчивости: медиана %.1f мм" % statistics.median(res["margins"]))
    print("  отставание сустава: медиана %.2f°" % statistics.median(res["track"]))
    print("  требуется по суставу %.0f °/с при разрешённых %.0f"
          % (res["summary"]["peak_rate_dps"], args.maxrate))
    if res["falls"]:
        print("  УПАЛ в мирах: " + "; ".join(res["falls"]))

    out = {"params": params, "maxrate": args.maxrate, "score": best[0],
           "grid_learn": GRID_LEARN, "grid_test": GRID_TEST,
           "test": {k: v for k, v in res.items() if k != "summary"},
           "peak_rate_dps": res["summary"]["peak_rate_dps"]}
    path = os.path.join(HERE, "results", "best_robust_%s.json" % tag)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print("\nзаписано: %s" % path)

    print()
    print("КОМАНДЫ НА РОБОТА (вывешенный, питание с ограничением тока):")
    print("    G 1")
    print("    H %.0f" % params["height"])
    print("    P %.0f" % params["span"])
    print("    C %.0f" % params["lift"])
    print("    V %.0f" % params["vel"])
    print("    W %.0f" % params["swing"])
    print("    B %.2f" % params["shiftgain"])
    print("    m %.0f" % args.maxrate)
    print("    p          <- предполётная; без неё не пускать")
    print()
    print("ПОСМОТРЕТЬ, ЧТО ПОЛУЧИЛОСЬ:")
    print("    python view.py --params %s"
          % os.path.relpath(path, os.getcwd()).replace("/", os.sep))
    print("СДЕЛАТЬ ПОЛЁТНУЮ КАРТУ:")
    print("    python export_set.py %s"
          % os.path.relpath(path, os.getcwd()).replace("/", os.sep))


if __name__ == "__main__":
    main()
