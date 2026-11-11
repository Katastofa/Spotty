"""
ОБРЫВ ПО ВЫСОТЕ: «не та походка» или «на этой высоте стоять нельзя»?

Протокол №2 §1 нашёл обрыв между H 150 и H 160 и оставил вопрос открытым:
ни опускание ЦТ, ни расширение стойки вердикт не переворачивали. Но тот свип
делался на РЫСИ и БЕЗ переноса корпуса — то есть мерил связку «высота +
эта конкретная походка», а не высоту.

Здесь свип разложен на три столбца, и разница между ними и есть ответ:

    trot                   как в §1, для сходимости с прошлым прогоном
    walk, перенос ВЫКЛ     та же высота, но статически устойчивая фазировка
    walk, перенос ВКЛ      плюс таблица прошивки ('B 1')

Если обрыв уезжает вверх от столбца к столбцу — дело было в походке.
Если стоит на месте — дело в высоте, и это вопрос к стойке, а не к настройкам.

ЧТО ЕЩЁ ПЕЧАТАЕТСЯ, и ради чего всё затевалось: рядом с вердиктом идут две
величины, которые разводят ДВЕ РАЗНЫЕ причины падения.

    запас    СРЕДНИЙ запас устойчивости, мм. Падает из-за ГЕОМЕТРИИ:
             проекция ЦТ вышла за опорный треугольник. Минимальный здесь
             не годится: в момент, когда все четыре стопы в воздухе (отскок
             при постановке, кувырок после падения), он вырождается в
             служебное -9000, и таблица из таких чисел ничего не измеряет.
    отст.    среднее отставание сустава от команды, град. Растёт из-за
             ПРИВОДА: момента не хватает, серво не доезжает, и робот
             оказывается не там, где считает генератор.

Лечатся они разным. Первое — переносом корпуса и фазировкой, второе — только
приседом, меньшим темпом или другими серво.

    python height.py                 три столбца, H 130…200
    python height.py --cycles 8      дольше прогон, строже вердикт
"""
import argparse
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E, model as M

HERE = os.path.dirname(os.path.abspath(__file__))

# Общая постановка для всех столбцов: меняется ТОЛЬКО высота и походка.
SPAN, VEL, LIFT = 25.0, 40.0, 15.0

# ОДИН ПРОГОН — НЕ ВЕРДИКТ, и это уже проверено на своей шкуре: расхождение
# 3e-5 градуса на входе переворачивает один прогон из восьми (Протокол №2
# §6.4). Поэтому каждая клетка таблицы — это ГОЛОСОВАНИЕ по маленькой сетке
# ПРИНЯТЫХ чисел. Масс у нас нет, значит высоту, на которой робот стоит
# ТОЛЬКО при одной удачной массе, объявлять рабочей нельзя.
GRID = [
    {},
    {"body_mass_kg": 0.85},
    {"body_mass_kg": 1.15},
    {"foot_friction": 0.7, "ground_friction": 0.7},
]


def case(gait, shift, height, cycles):
    p = {"height": height, "span": SPAN, "lift": LIFT, "vel": VEL}
    if gait == 3:
        p["swing"] = E.swing_for_duty(SPAN, VEL, 0.75)   # шаг: duty 0.75
    if shift:
        p["shiftgain"] = 1.0
    ok, dist, margins, track = 0, [], [], []
    for over in GRID:
        m, urdf, geom = E.prepare_urdf(HERE, gait=gait)
        for k, v in over.items():
            setattr(m, k, v)
        with open(urdf, "w", encoding="utf-8") as f:
            f.write(M.build_urdf(m, limits_deg=E._limits(geom)))
        E._SIM.clear()
        score, met, summ, full = E.evaluate(p, m, urdf, cycles=cycles, gait=gait)
        if not met["fell"]:
            ok += 1
        dist.append(met["x_m"] * 1000)
        margins.append(met["margin_mean_mm"])
        track.append(met["track_deg"])
    return ok, dist, margins, track


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=float, default=6.0)
    ap.add_argument("--from", dest="h0", type=float, default=130.0)
    ap.add_argument("--to", dest="h1", type=float, default=200.0)
    ap.add_argument("--step", type=float, default=10.0)
    ap.add_argument("--out", default=os.path.join(HERE, "results", "height.json"))
    args = ap.parse_args()

    m0, _, _ = E.prepare_urdf(HERE, gait=3)
    print("продольная база %.0f мм — %s" % (
        m0.base_long_mm,
        "ИЗМЕРЕНО (M4)" if m0.base_long_measured else "ПРИНЯТО, M4 не снят"))
    print("нули: M9 закрыт по нулевой позе, ошибка нулей в модели = 0")
    print("массы, ЦТ, момент серво — ПРИНЯТЫ, не взвешивалось")
    print()
    cols = [("trot", 0, False), ("walk", 3, False), ("walk+перенос", 3, True)]
    print("  %-7s | %s" % ("H, мм", " | ".join("%-26s" % c[0] for c in cols)))
    print("  %-7s | %s" % ("", " | ".join(
        "%-26s" % "устоял   путь  запас  отст." for c in cols)))
    print("  " + "-" * 98)

    out = []
    h = args.h0
    while h <= args.h1 + 0.01:
        row = {"height": h, "cols": {}}
        cells = []
        for name, gait, shift in cols:
            ok, dist, margins, track = case(gait, shift, h, args.cycles)
            cells.append("  %d/%d  %+6.0f %6.1f %6.2f" % (
                ok, len(GRID), statistics.median(dist),
                statistics.median(margins), statistics.median(track)))
            row["cols"][name] = {
                "ok": ok, "n": len(GRID), "dist": dist,
                "margin_mean_mm": margins, "track_deg": track,
            }
        print("  %-7.0f | %s" % (h, " | ".join(cells)))
        out.append(row)
        h += args.step

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print("\nзаписано: %s" % args.out)

    # --- где обрыв у каждого столбца -----------------------------------------
    print()
    for name, _, _ in cols:
        full_ok, any_ok = None, None
        for row in out:
            c = row["cols"][name]
            if c["ok"] == c["n"]:
                full_ok = row["height"]
            if c["ok"] > 0:
                any_ok = row["height"]
        print("  %-14s устоял ВЕЗДЕ до %s; хоть где-то до %s" % (
            name,
            ("%.0f мм" % full_ok) if full_ok else "НИ ОДНОЙ ВЫСОТЫ",
            ("%.0f мм" % any_ok) if any_ok else "НИ ОДНОЙ"))


if __name__ == "__main__":
    main()
