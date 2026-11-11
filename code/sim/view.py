"""
Просмотр походки с ползунками. Это и есть «видеть, как меняется походка».

    python view.py                     живые ползунки: крутите — походка меняется
    python view.py --params results/best_trot_seed1.json
    python view.py --csv results/traj_best_trot_seed1.csv     просто проиграть файл

КАК ЭТО РАБОТАЕТ. Ползунок двигает не картинку, а параметр ПРОШИВКИ. На
каждое изменение заново вызывается настоящий gait_test.ino, он выдаёт новую
траекторию 12 суставов, и физика проигрывает её. Поэтому всё, что видно в
окне, можно ввести с клавиатуры в робота: span -> 'P', lift -> 'C',
vel -> 'V', swing -> 'W', height -> 'H', shiftx -> 'x'.

На Windows для живых ползунков нужен bin/trace.exe (лежит рядом). Если его
нет и собрать нечем, остаётся режим --csv: проиграть готовый файл.
"""
import argparse
import json
import os
import sys
import time

import pybullet as p

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E, firmware, model as M
from spotty_sim.player import Sim, read_csv

HERE = os.path.dirname(os.path.abspath(__file__))
GAITS = ["trot", "pace", "bound", "walk", "gallop"]
ORDER = ["span", "lift", "vel", "swing", "height", "shiftx",
         "shiftgain", "maxrate"]
# ПОДПИСИ ПОЛЗУНКОВ — ЛАТИНИЦЕЙ, И ЭТО НЕ НЕБРЕЖНОСТЬ.
# Отладочный слой PyBullet рисует текст своим растровым шрифтом, в котором
# кириллицы нет: русская подпись превратится в пустое место или в квадраты,
# и оператор не поймёт, какой ползунок за что отвечает. В консоль при этом
# по-прежнему печатается по-русски — там шрифт системный.
# В скобках — буква команды на роботе, чтобы ползунок и клавиатура читались
# как одно и то же.
LABEL = {"span": "span mm  (P)", "lift": "lift mm  (C)",
         "vel": "vel mm/s  (V)", "swing": "swing ms  (W)",
         "height": "height mm  (H)", "shiftx": "body fwd mm  (X)",
         "shiftgain": "body shift 0=off  (B)",
         "maxrate": "joint rate limit deg/s  (m)"}

# Два ползунка, которых в SPACE нет и не должно быть: это не параметры
# походки, а то, ЧЕМ её исполняют.
#
#   shiftgain  доля геометрически нужного переноса корпуса. На роботе это
#              команда 'B'. 0 — как было до 2026-09-12.
#   maxrate    предохранитель прошивки, команда 'm'. Смотреть на него стоит:
#              траектории по умолчанию требуют 130…200 °/с, а разрешено 100,
#              то есть ограничитель режет команду и робот идёт НЕ ТОЙ
#              походкой, которую посчитал генератор. Подняв ползунок, это
#              видно глазами и в строке «пик N из M °/с».
EXTRA_RANGE = {"shiftgain": (0.0, 1.5), "maxrate": (40.0, 260.0)}
EXTRA_START = {"shiftgain": 0.0, "maxrate": 100.0, "shiftx": 0.0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=None, help="проиграть готовый файл")
    ap.add_argument("--params", default=None, help="стартовые параметры из JSON")
    ap.add_argument("--gait", type=int, default=3)   # 3 = walk, там виден перенос
    ap.add_argument("--cycles", type=float, default=3.0)
    ap.add_argument("--base-long", type=float, default=None)
    args = ap.parse_args()

    start = dict(E.DEFAULTS)
    start.update(EXTRA_START)
    gait = args.gait
    if args.params:
        with open(args.params, "r", encoding="utf-8") as f:
            blob = json.load(f)
        start.update(blob.get("params", blob))
        gait = blob.get("gait", gait)

    # У КАЖДОГО ползунка обязано быть стартовое значение, даже если в файле
    # набора его нет: наборы бывают из разных пространств поиска (в robust,
    # например, нет shiftx), и отсутствие ключа роняло бы окно на ровном
    # месте. Чего нет — ставим в середину диапазона и зажимаем в границы.
    for k in ORDER:
        lo, hi = _range(k)
        start[k] = min(max(float(start.get(k, 0.5 * (lo + hi))), lo), hi)

    m, urdf, geom = E.prepare_urdf(HERE, base_long_mm=args.base_long)
    print("продольная база %.0f мм — %s" % (
        m.base_long_mm,
        "ИЗМЕРЕНО (M4, рулетка)" if m.base_long_measured
        else "ПРИНЯТО, M4 не снят"))
    sim = Sim(m, urdf, gui=True)

    if args.csv:
        ts, rows, settle = read_csv(args.csv)
        print("проигрываю %s (%d строк)" % (args.csv, len(rows)))
        while True:
            sim.run(ts, rows, settle, start["height"], realtime=True)
            time.sleep(0.5)

    # --- живые ползунки -----------------------------------------------------
    sl = {k: p.addUserDebugParameter(LABEL[k], *(_range(k) + (start[k],)),
                                     physicsClientId=sim.cid) for k in ORDER}
    sl_gait = p.addUserDebugParameter("gait 0trot 1pace 2bound 3walk 4gallop",
                                      0, 4, gait, physicsClientId=sim.cid)
    btn = p.addUserDebugParameter("restart", 1, 0, 0, physicsClientId=sim.cid)
    last_btn = p.readUserDebugParameter(btn, physicsClientId=sim.cid)

    cur, label = None, None
    while True:
        want = {k: round(p.readUserDebugParameter(sl[k], physicsClientId=sim.cid), 2)
                for k in ORDER}
        want_gait = int(round(p.readUserDebugParameter(sl_gait, physicsClientId=sim.cid)))
        b = p.readUserDebugParameter(btn, physicsClientId=sim.cid)
        if cur != (want, want_gait) or b != last_btn:
            last_btn = b
            cur = (dict(want), want_gait)
            summ, csv = firmware.trace(want, cycles=args.cycles, gait=want_gait)
            ts, rows, settle = read_csv(csv)
            os.remove(csv)
            clip = summ["peak_rate_dps"] > summ["max_rate_dps"] + 0.5
            shift_mm = summ.get("shift_amp_mm", 0.0) * want["shiftgain"]
            # На экран — латиницей (шрифт PyBullet), в консоль — по-русски.
            txt = ("%s | T %.0f ms | duty %.2f | peak %.0f of %.0f deg/s%s%s"
                   " | body shift %+.1f mm"
                   % (summ["gait"], summ["cycle_ms"], summ["duty"],
                      summ["peak_rate_dps"], summ["max_rate_dps"],
                      "  <- CLIPPED" if clip else "",
                      "  IK FAIL" if summ.get("ik_fail") else "", shift_mm))
            print("%s | период %.0f мс | duty %.2f | пик %.0f из %.0f °/с%s%s"
                  " | перенос %+.1f мм"
                  % (summ["gait"], summ["cycle_ms"], summ["duty"],
                     summ["peak_rate_dps"], summ["max_rate_dps"],
                     "  <- РЕЖЕТ" if clip else "",
                     "  ОТКАЗ IK" if summ.get("ik_fail") else "", shift_mm),
                  flush=True)
            if label is not None:
                p.removeUserDebugItem(label, physicsClientId=sim.cid)
            label = p.addUserDebugText(txt, [0, 0, 0.42], textColorRGB=[0, 0, 0],
                                       textSize=1.2, physicsClientId=sim.cid)
        met = sim.run(ts, rows, settle, want["height"], tick_ms=summ["tick_ms"],
                      realtime=True)
        print("  прошёл %+.0f мм, снос %+.0f мм, крен СКЗ %.1f°, %s"
              % (met["x_m"] * 1000, met["y_m"] * 1000, met["roll_rms_deg"],
                 "УПАЛ" if met["fell"] else "устоял"), flush=True)


def _range(k):
    if k in EXTRA_RANGE:
        return EXTRA_RANGE[k]
    lo, hi = E.SPACE[k]
    return (lo, hi)


if __name__ == "__main__":
    main()
