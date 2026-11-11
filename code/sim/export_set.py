"""
ПОЛЁТНАЯ КАРТА: из результата обучения — в то, что вводится с клавиатуры.

Результат обучения лежит в JSON и оператору бесполезен: в него нельзя ткнуть
пальцем, стоя у робота с терминалом. Этот скрипт превращает его в две вещи:

    ПОЛЁТНАЯ_КАРТА_<тег>.md   что это за набор, чем проверен, чего ждать,
                              что делать, если пойдёт не так
    set_<тег>.txt             голые команды по одной в строке — вставить
                              в терминал целиком

Команды выводятся В ТОМ ЖЕ ПОРЯДКЕ, в каком их обязан принять робот: сперва
движок, потом параметры, потом предполётная, и только потом — движение.
Порядок не косметический: 'H', 'P', 'C' и прочие в декартовом режиме имеют
смысл только при 'G 1', а каждая смена параметра сама гоняет предполётную.

    python export_set.py results/best_robust_m100.json
    python export_set.py results/best_robust_m100.json --cycles 3
"""
import argparse
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E, firmware

HERE = os.path.dirname(os.path.abspath(__file__))

# Параметр -> команда прошивки. Порядок списка = порядок ввода.
CMD = [
    ("engine",    "G 1",      "декартов движок (мм), а не угловой"),
    ("height",    "H %.0f",   "глубина стопы под осью хипа"),
    ("span",      "P %.0f",   "полушаг: полный шаг вдвое больше"),
    ("lift",      "C %.0f",   "подъём стопы в переносе"),
    ("vel",       "V %.0f",   "скорость корпуса; из неё выводится опора"),
    ("swing",     "W %.0f",   "время переноса; вместе с V даёт duty и период"),
    ("shiftx",    "X %.0f",   "постоянный сдвиг корпуса вперёд"),
    ("shiftgain", "B %.2f",   "перенос корпуса по фазе, доля расчётной амплитуды"),
    ("maxrate",   "m %.0f",   "предел скорости сустава"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("best", help="results/best_*.json")
    ap.add_argument("--gait", type=int, default=3)
    ap.add_argument("--cycles", type=float, default=4.0)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--outdir", default=HERE)
    args = ap.parse_args()

    with open(args.best, "r", encoding="utf-8") as f:
        blob = json.load(f)
    params = dict(blob.get("params", blob))
    tag = args.tag or os.path.splitext(os.path.basename(args.best))[0]

    # Прогоняем набор через НАСТОЯЩУЮ прошивку — числа в карте должны быть её,
    # а не пересказом. Заодно это проверка, что набор вообще считается.
    summ, csv = firmware.trace(params, gait=args.gait, cycles=args.cycles)
    os.remove(csv)

    test = blob.get("test") or {}
    lines = []
    for key, fmt, _why in CMD:
        if key == "engine":
            lines.append("G 1")
        elif key in params:
            lines.append(fmt % params[key])
    lines.append("p")

    txt = os.path.join(args.outdir, "set_%s.txt" % tag)
    with open(txt, "w", encoding="utf-8", newline="\r\n") as f:
        f.write("\n".join(lines) + "\n")

    md = os.path.join(args.outdir, "ПОЛЁТНАЯ_КАРТА_%s.md" % tag)
    with open(md, "w", encoding="utf-8") as f:
        w = f.write
        w("# ПОЛЁТНАЯ КАРТА — набор `%s`\n\n" % tag)
        w("Сгенерировано `export_set.py` из `%s`.\n"
          "Числа ниже посчитаны ПРОШИВКОЙ, а не пересказаны.\n\n"
          % os.path.basename(args.best))

        w("## 1. Что это за набор\n\n")
        w("| параметр | значение | команда | что значит |\n|---|---|---|---|\n")
        for key, fmt, why in CMD:
            if key == "engine":
                w("| движок | декартов | `G 1` | %s |\n" % why)
            elif key in params:
                w("| `%s` | %.2f | `%s` | %s |\n"
                  % (key, params[key], fmt % params[key], why))
        w("\n")
        w("Походка **%s**, период **%.0f мс**, duty **%.2f** "
          "(опора %.0f %% цикла).\n\n"
          % (summ["gait"], summ["cycle_ms"], summ["duty"], summ["duty"] * 100))

        w("## 2. Чем этот набор проверен\n\n")
        if test:
            w("Прогнан по **%d мирам** с разными ПРИНЯТЫМИ массами, ЦТ, трением,\n"
              "жёсткостью контура и латентностью — устоял в **%d**.\n\n"
              % (test.get("n", len(test.get("dist", []))), test.get("ok", 0)))
            if test.get("dist"):
                w("* путь: медиана **%+.0f мм**, худший **%+.0f мм**\n"
                  % (statistics.median(test["dist"]), min(test["dist"])))
            if test.get("margins"):
                w("* запас устойчивости: медиана **%.1f мм**\n"
                  % statistics.median(test["margins"]))
            if test.get("falls"):
                w("* **упал** в мирах: %s\n" % "; ".join(test["falls"]))
            w("\n")
        w("Пик по суставу **%.0f °/с** при разрешённых **%.0f**%s.\n\n"
          % (summ["peak_rate_dps"], summ["max_rate_dps"],
             " — ОГРАНИЧИТЕЛЬ РЕЖЕТ, походка пойдёт не та"
             if summ["peak_rate_dps"] > summ["max_rate_dps"] + 0.5
             else ", ограничитель не вмешивается"))
        w("Перенос корпуса: амплитуда из геометрии **%.2f мм**, "
          "доля **%.2f** → **%+.2f мм**.\n\n"
          % (summ.get("shift_amp_mm", 0.0), params.get("shiftgain", 0.0),
             summ.get("shift_amp_mm", 0.0) * params.get("shiftgain", 0.0)))
        w("Продольная база: **%s**.\n\n" % summ.get("base_long_source", "?"))

        w("> **Это НЕ рекомендация для пола.** Набор найден в модели, у которой\n"
          "> массы, ЦТ и модель серво — ПРИНЯТЫ, а не измерены. Первый прогон\n"
          "> только на подвесе, по `SAFETY.md`.\n\n")

        w("## 3. Порядок ввода\n\n```\n%s\n```\n\n" % "\n".join(lines))
        w("Готовый файл для вставки целиком: `set_%s.txt`.\n\n" % tag)
        w("Порядок не косметический: `H`, `P`, `C`, `V`, `W`, `X`, `B` в\n"
          "декартовом режиме имеют смысл только после `G 1`, а каждая смена\n"
          "параметра сама гоняет предполётную. Последняя `p` — контрольная.\n\n")

        w("## 4. Что обязано напечатать `p`\n\n")
        w("* `Предполётная проверка пройдена (пределы).` — иначе НЕ ПУСКАТЬ;\n")
        w("* пик требуемой скорости **%.0f °/с** — если робот показал другое\n"
          "  число, значит применились не те параметры;\n" % summ["peak_rate_dps"])
        w("* строку переноса корпуса с амплитудой **%.2f мм**;\n"
          % (summ.get("shift_amp_mm", 0.0) * params.get("shiftgain", 0.0)))
        w("* ни одного `[!!]`.\n\n")

        w("## 5. Прогон и запись\n\n")
        w("```\nn          выйти в позу (движение! робот должен быть на подвесе)\n"
          "w %d        пойти\n"
          "x          остановить в любой момент\n```\n\n" % int(args.cycles))
        w("**Сохраните вывод терминала в файл целиком.** Он нужен для сверки:\n\n")
        w("```\npython compare_real.py <лог.txt> --params %s\n```\n\n"
          % os.path.relpath(args.best, HERE).replace("\\", "/"))
        w("Сверка покажет, совпали ли углы, которые робот РЕАЛЬНО выдал, с теми,\n"
          "которые посчитала модель. Расхождение означает, что на роботе\n"
          "применилось не то, что в модели, — и до выяснения дальше идти нельзя.\n\n")

        w("## 6. Если пойдёт не так\n\n")
        w("| что видно | что это | что делать |\n|---|---|---|\n")
        w("| `[!!] ОТКАЗ IK на ходу` | заказанная точка недостижима | `x`, "
          "проверить `H` и `P` |\n")
        w("| `АВАРИЙНЫЙ СТОП` по пределу | сустав упёрся в жёсткий предел | "
          "`x`, смотреть, какой канал, сверить нули |\n")
        w("| серво гудят, ноги отстают | не хватает момента или скорости | "
          "`x`, снизить `V`, поднять `W` |\n")
        w("| робот валится на борт | запас устойчивости ушёл | "
          "`x`, поднять `B`, снизить `H` |\n")
        w("| сверка даёт большое расхождение | применились не те параметры или "
          "нули | не гонять дальше, разбираться |\n")
    print("записано:")
    print("  %s" % md)
    print("  %s" % txt)
    print()
    print("\n".join(lines))


if __name__ == "__main__":
    main()
