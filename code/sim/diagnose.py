"""
Почему модель падает на умолчаниях прошивки.

Лесенка абляций: берём умолчания (H 190) и меняем РОВНО ОДНО за раз. Что
переворачивает вердикт «упал» — то и причина. Что не переворачивает — то не
причина, сколько бы правдоподобно ни звучало.

    python diagnose.py                 рысь и шаг
    python diagnose.py --gait 3        только шаг
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E, model as M

HERE = os.path.dirname(os.path.abspath(__file__))
GAIT = {0: "trot", 3: "walk"}


def swing_for_duty(span, vel, duty):
    """Период и duty в декартовом движке ВЫВОДЯТСЯ. Хочешь duty — считай swing."""
    t_stance = 2.0 * span / vel          # с
    return 1000.0 * t_stance * (1.0 - duty) / duty


def run(name, gait, params=None, model_over=None, cycles=3.0):
    m, urdf, geom = E.prepare_urdf(HERE)
    for k, v in (model_over or {}).items():
        setattr(m, k, v)
    with open(urdf, "w", encoding="utf-8") as f:
        f.write(M.build_urdf(m, limits_deg=E._limits(geom)))
    E._SIM.clear()
    score, met, summ, full = E.evaluate(params or {}, m, urdf, cycles=cycles, gait=gait)
    print("  %-38s %+7.0f %7.2f %8.1f %6s"
          % (name, met["x_m"] * 1000, met["roll_rms_deg"],
             met["margin_min_mm"], "ДА" if met["fell"] else "нет"))
    return met


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gait", type=int, default=None)
    ap.add_argument("--cycles", type=float, default=3.0)
    args = ap.parse_args()
    gaits = [args.gait] if args.gait is not None else [0, 3]

    D = E.DEFAULTS
    for gait in gaits:
        print("\n=== %s, всё от умолчаний прошивки H %.0f ==="
              % (GAIT.get(gait, gait), D["height"]))
        print("  %-38s %7s %7s %8s %6s"
              % ("что поменяли", "путь,мм", "крен°", "запас,мм", "упал"))
        run("ничего (умолчания)", gait)

        print("  --- походка ---")
        run("шаг на месте (span 1)", gait, {"span": 1.0})
        run("без подъёма стопы (lift 1)", gait, {"lift": 1.0})
        run("вдвое медленнее (vel 30)", gait, {"vel": 30.0})
        run("duty 0.75 (swing выводится)", gait,
            {"swing": swing_for_duty(D["span"], D["vel"], 0.75)})
        run("присед H 150", gait, {"height": 150.0})
        run("присед H 150 + duty 0.75", gait,
            {"height": 150.0, "swing": swing_for_duty(D["span"], D["vel"], 0.75)})

        print("  --- геометрия и массы (ПРИНЯТО) ---")
        run("ЦТ корпуса ниже на 50 мм", gait, None, {"body_com_z_mm": -50.0})
        run("ЦТ корпуса ниже на 90 мм", gait, None, {"body_com_z_mm": -90.0})
        run("стойка шире на 40 мм", gait, None, {"base_lat_mm": 118.0})
        run("стойка шире на 80 мм", gait, None, {"base_lat_mm": 158.0})

        print("  --- привод (ПРИНЯТО) ---")
        run("жёстче контур kp 30", gait, None, {"servo_kp": 30.0})
        run("мягче контур kp 12", gait, None, {"servo_kp": 12.0})
        run("латентность 0", gait, None, {"latency_ms": 0.0})
        run("трение 1.4", gait, None, {"foot_friction": 1.4, "ground_friction": 1.4})

    print("\nПояснение к колонке «запас»: знаковое расстояние от проекции ЦТ до")
    print("границы опорного многоугольника. У рыси опора — отрезок из двух стоп,")
    print("там запас неположителен ВСЕГДА, и это свойство рыси, а не поломка.")


if __name__ == "__main__":
    main()
