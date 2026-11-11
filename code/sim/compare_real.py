"""
СВЕРКА РОБОТА С МОДЕЛЬЮ по логу терминала.

Робот во время `w` печатает пять раз в секунду то, что РЕАЛЬНО ушло в серво:

    walk   c2/6 ph=0.37 | FR T -45.7 K  62.0 | FL T ... | BR ... | BL ...

Это углы ПОСЛЕ клампа по жёсткому пределу и ПОСЛЕ ограничителя скорости —
ровно то же, что кладёт в CSV стенд. Значит их можно сравнить число в число.

ЧТО ЭТА СВЕРКА ПРОВЕРЯЕТ. Что прошивка в роботе посчитала ТО ЖЕ САМОЕ, что
модель: те же нули и пределы из spotty_config.h, ту же геометрию, те же
параметры — то есть что оператор действительно ввёл то, что собирался, и что
во флеше не лежит забытая накладка. Это предполётная сверка состояния.

ЧЕГО ОНА НЕ ПРОВЕРЯЕТ, и путать нельзя: дошло ли серво туда, куда ему велели.
Обратной связи по положению в проекте нет — робот печатает КОМАНДУ, а не
факт. Физику проверяет глаз оператора и, позже, замеры.

    python compare_real.py log.txt --params results/best_robust_m100.json
    python compare_real.py log.txt --params ... --csv out.csv
"""
import argparse
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import firmware

HERE = os.path.dirname(os.path.abspath(__file__))
LEGS = ("FR", "FL", "BR", "BL")

# walk   c2/6 ph=0.37 | FR T -45.7 K  62.0 | FL T ...
RE_HEAD = re.compile(r"^\s*(\w+)\s+c(\d+)/(\d+)\s+ph=([\d.]+)")
RE_LEG = re.compile(r"\|\s*(FR|FL|BR|BL)\s+T\s*(-?[\d.]+)\s+K\s*(-?[\d.]+)")


def parse_log(path):
    """Лог терминала -> список (фаза, {нога: (бедро, колено)})."""
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            h = RE_HEAD.match(line)
            if not h:
                continue
            legs = {m.group(1): (float(m.group(2)), float(m.group(3)))
                    for m in RE_LEG.finditer(line)}
            if len(legs) != 4:
                continue
            rows.append((float(h.group(4)), legs, int(h.group(2))))
    return rows


def read_traj(path):
    """CSV стенда -> эталонный ЦИКЛ: [(фаза, 12 углов)], отсортированный.

    Свой читатель, а не player.read_csv: тому фаза не нужна, и он её
    выбрасывает, а здесь сверка идёт ИМЕННО ПО ФАЗЕ — робот печатает её сам,
    и это единственное, что надёжно связывает две записи во времени.

    БЕРЁТСЯ ПОСЛЕДНИЙ ПОЛНЫЙ ЦИКЛ, а не вся запись. Первые циклы после выхода
    из позы — переходный процесс: ограничитель скорости догоняет траекторию,
    и углы там отличаются от установившихся на несколько градусов. Сверять
    установившийся цикл робота с переходным циклом стенда — значит увидеть
    расхождение, которого нет.
    """
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        f.readline()                        # заголовок
        for line in f:
            v = [float(x) for x in line.split(",")]
            if v[13] < 0:                   # стояние в позе, фазы ещё нет
                continue
            rows.append((v[13], v[1:13]))
    if not rows:
        return []
    # Границы циклов — там, где фаза перескакивает назад.
    starts = [0] + [i for i in range(1, len(rows)) if rows[i][0] < rows[i - 1][0]]
    if len(starts) >= 2:
        rows = rows[starts[-2]:starts[-1]]   # последний ЗАВЕРШЁННЫЙ цикл
    return sorted(rows, key=lambda r: r[0])


def at_phase(traj, ph):
    """Линейная интерполяция эталонного цикла по фазе. Фаза циклическая."""
    n = len(traj)
    if n == 1:
        return traj[0][1]
    lo = 0
    while lo + 1 < n and traj[lo + 1][0] <= ph:
        lo += 1
    hi = (lo + 1) % n
    p0, a0 = traj[lo]
    p1, a1 = traj[hi]
    d = (p1 - p0) % 1.0
    t = 0.0 if d <= 1e-9 else (((ph - p0) % 1.0) / d)
    t = min(max(t, 0.0), 1.0)
    return [a0[j] + (a1[j] - a0[j]) * t for j in range(12)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="файл с сохранённым выводом терминала")
    ap.add_argument("--params", required=True, help="results/best_*.json")
    ap.add_argument("--gait", type=int, default=3)
    ap.add_argument("--cycles", type=float, default=6.0)
    ap.add_argument("--csv", default=None, help="куда выгрузить построчную сверку")
    args = ap.parse_args()

    with open(args.params, "r", encoding="utf-8") as f:
        blob = json.load(f)
    params = dict(blob.get("params", blob))

    real = parse_log(args.log)
    # Первый цикл робота — тоже переходный: он только что вышел из позы.
    if any(c > 1 for _, _, c in real):
        real = [r for r in real if r[2] > 1]
    if not real:
        print("В логе нет ни одной строки походки.")
        print("Ожидается формат:  walk   c1/6 ph=0.05 | FR T -45.7 K  62.0 | ...")
        print("Это то, что робот печатает во время 'w'. Сохраните вывод целиком.")
        return 2

    summ, csv = firmware.trace(params, gait=args.gait, cycles=args.cycles)
    traj = read_traj(csv)
    os.remove(csv)
    if not traj:
        print("Стенд не дал ходовой части — проверьте набор параметров.")
        return 2

    # ДОПУСК СЧИТАЕТСЯ, А НЕ НАЗНАЧАЕТСЯ. Две неустранимые причины разойтись
    # даже при полностью совпавших числах:
    #   1. робот печатает фазу с двумя знаками, то есть с точностью +-0.005
    #      цикла; на пике скорости это peak * 0.005 * период градусов;
    #   2. мёртвая зона серво 3 мкс = 0.54 градуса.
    # Всё, что больше их суммы, округлением уже не объясняется.
    tol = (summ["peak_rate_dps"] * 0.005 * summ["cycle_ms"] / 1000.0) + 0.54

    print("СВЕРКА РОБОТА С МОДЕЛЬЮ")
    print("  лог:      %s  (%d строк походки)" % (args.log, len(real)))
    print("  набор:    %s" % args.params)
    print("  походка:  %s, период %.0f мс, duty %.2f"
          % (summ["gait"], summ["cycle_ms"], summ["duty"]))
    print("  эталон:   последний полный цикл стенда, %d точек" % len(traj))
    print("  допуск:   %.2f° (печать фазы %.2f° + мёртвая зона 0.54°)"
          % (tol, tol - 0.54))
    print()

    # 12 столбцов CSV идут по ногам FR, FL, BR, BL, в каждой hip, thigh, knee.
    idx = {leg: (3 * i + 1, 3 * i + 2) for i, leg in enumerate(LEGS)}  # бедро, колено
    acc = {(leg, j): [] for leg in LEGS for j in ("бедро", "колено")}
    out_rows = []
    for ph, legs, cyc in real:
        e = at_phase(traj, ph)
        for leg in LEGS:
            it, ik = idx[leg]
            dT = legs[leg][0] - e[it]
            dK = legs[leg][1] - e[ik]
            acc[(leg, "бедро")].append(dT)
            acc[(leg, "колено")].append(dK)
            out_rows.append((cyc, ph, leg, legs[leg][0], e[it], dT,
                             legs[leg][1], e[ik], dK))

    print("  %-6s %-8s %9s %9s %9s" % ("нога", "сустав", "СКО, °", "макс, °", "смещение, °"))
    worst = 0.0
    for leg in LEGS:
        for j in ("бедро", "колено"):
            d = acc[(leg, j)]
            rms = (sum(v * v for v in d) / len(d)) ** 0.5
            mx = max(abs(v) for v in d)
            bias = sum(d) / len(d)
            worst = max(worst, mx)
            print("  %-6s %-8s %9.2f %9.2f %+9.2f" % (leg, j, rms, mx, bias))

    print()
    if worst < tol:
        print("  СОВПАЛО. Худшее расхождение %.2f° при допуске %.2f°."
              % (worst, tol))
        print("  Прошивка в роботе считает то же, что модель.")
    else:
        print("  РАСХОЖДЕНИЕ %.2f° при допуске %.2f°. Округлением не объясняется."
              % (worst, tol))
        print("  Смотреть в таком порядке:")
        print("    1. Все ли параметры введены? Сверьте 's' на роботе с картой.")
        print("    2. Нет ли накладки нулей во флеше? В servo_calib: 'hist', 'lim'.")
        print("    3. Та ли прошивка залита? Сверьте вывод 'p' с полётной картой.")
        print("    4. Смещение одного знака у одного сустава — это его нуль.")

    if args.csv:
        import csv as _csv
        with open(args.csv, "w", encoding="utf-8", newline="") as f:
            wr = _csv.writer(f)
            wr.writerow(["цикл", "фаза", "нога", "бедро_робот", "бедро_модель",
                         "бедро_разн", "колено_робот", "колено_модель", "колено_разн"])
            wr.writerows(out_rows)
        print("\n  построчная сверка: %s" % args.csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
