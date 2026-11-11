"""
Свип по ПРИНЯТЫМ числам: насколько вывод держится на том, чего мы не мерили.

Вывод, который меняется на противоположный при переходе продольной базы со
135 на 232 мм, — это не вывод, а свойство выдуманного числа. Здесь это видно
явно, поэтому отчёт не может об этом умолчать.

    python sensitivity.py --params results/best_trot_seed1.json
"""
import argparse
import copy
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E, model as M

HERE = os.path.dirname(os.path.abspath(__file__))


def run(params, gait, cycles, **over):
    m, urdf, geom = E.prepare_urdf(HERE, base_long_mm=over.pop("base_long_mm", None))
    for k, v in over.items():
        setattr(m, k, v)
    # Массы и геометрия сидят в URDF, а не читаются на ходу: после подмены
    # файл надо пересобрать, иначе свип молча ничего не меняет.
    with open(urdf, "w", encoding="utf-8") as f:
        f.write(M.build_urdf(m, limits_deg=E._limits(geom)))
    E._SIM.clear()
    return E.evaluate(params, m, urdf, cycles=cycles, gait=gait)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", required=True)
    ap.add_argument("--cycles", type=float, default=4.0)
    ap.add_argument("--out", default=os.path.join(HERE, "results", "sensitivity.json"))
    args = ap.parse_args()
    with open(args.params, "r", encoding="utf-8") as f:
        blob = json.load(f)
    params, gait = blob["params"], blob.get("gait", 0)

    sweeps = [
        ("продольная база, мм (ИЗМЕРЕНО 186, M4)", "base_long_mm",
         [180.0, 186.0, 192.0]),   # +-6 мм — ошибка рулетки, а не коридор STL
        ("трение стопы (ПРИНЯТО)", "foot_friction", [0.5, 0.9, 1.4]),
        ("масса корпуса, кг (ПРИНЯТО)", "body_mass_kg", [0.7, 0.98, 1.4]),
        ("жёсткость контура (ПРИНЯТО)", "servo_kp", [12.0, 20.0, 30.0]),
        ("латентность, мс (ПРИНЯТО)", "latency_ms", [3.0, 12.0, 30.0]),
        ("момент серво, Н·м (ПРИНЯТО)", "stall_torque_nm", [1.5, 3.92, 6.0]),
    ]
    out = []
    print("%-38s %8s %9s %8s %7s %6s" % ("что меняем", "значение", "прошёл,мм",
                                         "снос,мм", "крен°", "упал"))
    for title, attr, values in sweeps:
        for v in values:
            over = {attr: v}
            if attr == "foot_friction":
                over["ground_friction"] = v
            score, met, summ, full = run(params, gait, args.cycles, **over)
            print("%-38s %8.2f %9.0f %8.0f %7.2f %6s"
                  % (title, v, met["x_m"] * 1000, met["y_m"] * 1000,
                     met["roll_rms_deg"], "ДА" if met["fell"] else "нет"))
            out.append({"what": title, "attr": attr, "value": v, "score": score,
                        "metrics": met})
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print("записано: %s" % args.out)


if __name__ == "__main__":
    main()
