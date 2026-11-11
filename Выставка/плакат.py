# -*- coding: utf-8 -*-
"""
Плакат A2 для выставочного стенда.

    python плакат.py                          адрес пульта по умолчанию
    python плакат.py --url http://192.168.1.50:8080/   свой адрес
    python плакат.py --no-qr                  без QR (если пульта не будет)

Нужны: matplotlib, segno (pip install matplotlib segno).
Результат: ПЛАКАТ_A2.pdf (вектор, для печати) и ПЛАКАТ_A2.png (посмотреть).
"""
import argparse
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.font_manager import FontProperties, findfont

MM = 1.0 / 25.4
W_MM, H_MM = 420.0, 594.0                    # A2 портрет

PAPER = "#f4f1ea"
INK = "#161a20"
GRAY = "#8d9299"
SIGNAL = "#1b62d6"
LEG = {"FR": "#d2563f", "FL": "#3499c4", "BR": "#d9a227", "BL": "#6a9e4b"}


def font(weight="normal", stretch="normal"):
    fp = FontProperties(family="DejaVu Sans", weight=weight, stretch=stretch)
    findfont(fp)                              # упадёт заметно, если шрифта нет
    return fp


def rounded(ax, x, y, w, h, r=3, **kw):
    ax.add_patch(patches.FancyBboxPatch(
        (x + r, y + r), w - 2 * r, h - 2 * r,
        boxstyle="round,pad=%f" % r, linewidth=kw.pop("lw", 1.2), **kw))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.137.1:8080/")
    ap.add_argument("--no-qr", action="store_true")
    ap.add_argument("--out", default="ПЛАКАТ_A2")
    args = ap.parse_args()

    reg, bold, light = font(), font("bold"), font(stretch="condensed")

    fig = plt.figure(figsize=(W_MM * MM, H_MM * MM))
    fig.patch.set_facecolor(PAPER)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W_MM)
    ax.set_ylim(0, H_MM)
    ax.axis("off")
    ax.set_facecolor(PAPER)

    M = 30.0                                   # поле

    # --- шапка -------------------------------------------------------------
    ax.text(M, H_MM - 58, "SPOTTY", fontproperties=bold, fontsize=96,
            color=INK, va="baseline")
    ax.text(M, H_MM - 74, "ЧЕТВЕРОНОГИЙ  ШАГАЮЩИЙ  РОБОТ",
            fontproperties=light, fontsize=15, color=GRAY, va="baseline",
            linespacing=1.4)
    ax.plot([M, W_MM - M], [H_MM - 86, H_MM - 86], color=INK, lw=1.0)
    ax.text(W_MM - M, H_MM - 58, "01", fontproperties=light, fontsize=15,
            color=GRAY, ha="right", va="baseline")

    # --- схема: ESP32-S3 -> PCA9685 -> 12 каналов --------------------------
    cx = W_MM / 2.0
    top = H_MM - 120.0

    rounded(ax, cx - 52, top - 26, 104, 26, r=3, facecolor="none", edgecolor=INK)
    ax.text(cx, top - 10, "ESP32-S3", fontproperties=bold, fontsize=19,
            color=INK, ha="center", va="center")
    ax.text(cx, top - 33, "походку считает контроллер",
            fontproperties=light, fontsize=11, color=GRAY, ha="center")

    y_i2c_a, y_i2c_b = top - 40, top - 70
    ax.annotate("", xy=(cx, y_i2c_b), xytext=(cx, y_i2c_a),
                arrowprops=dict(arrowstyle="-|>", color=SIGNAL, lw=1.6,
                                mutation_scale=14))
    ax.text(cx + 6, (y_i2c_a + y_i2c_b) / 2, "I²C", fontproperties=reg,
            fontsize=12, color=SIGNAL, va="center")

    rounded(ax, cx - 52, y_i2c_b - 26, 104, 26, r=3, facecolor="none",
            edgecolor=INK)
    ax.text(cx, y_i2c_b - 13, "PCA9685", fontproperties=bold, fontsize=19,
            color=INK, ha="center", va="center")
    ax.text(cx, y_i2c_b - 36, "12 каналов ШИМ", fontproperties=light,
            fontsize=11, color=GRAY, ha="center")

    # веер из 12 отводов
    bus_y = y_i2c_b - 48
    fan_y = bus_y - 40
    half = 132.0
    ax.plot([cx - half, cx + half], [bus_y, bus_y], color=INK, lw=1.0)
    ax.plot([cx, cx], [y_i2c_b - 26, bus_y], color=INK, lw=1.0)

    order = ["FL", "FR", "BL", "BR"]
    names = {"FL": "ЛЕВАЯ ПЕРЕДНЯЯ", "FR": "ПРАВАЯ ПЕРЕДНЯЯ",
             "BL": "ЛЕВАЯ ЗАДНЯЯ", "BR": "ПРАВАЯ ЗАДНЯЯ"}
    ch = 0
    for gi, leg in enumerate(order):
        gx = cx - half + (2 * gi + 1) * half / 4.0
        for j in range(3):
            x = gx + (j - 1) * 13.0
            ax.plot([x, x], [bus_y, fan_y], color=LEG[leg], lw=1.4)
            ax.add_patch(patches.Circle((x, fan_y), 1.6, color=LEG[leg]))
            ax.text(x, fan_y - 7, "%02d" % ch, fontproperties=light,
                    fontsize=9, color=GRAY, ha="center")
            ch += 1
        ax.plot([gx - 16, gx + 16], [fan_y - 14, fan_y - 14],
                color=LEG[leg], lw=2.6, solid_capstyle="butt")
        ax.text(gx, fan_y - 20, names[leg], fontproperties=light, fontsize=9.5,
                color=INK, ha="center", va="top")

    ax.text(cx, fan_y - 36, "КАЖДАЯ НОГА — ТРИ СУСТАВА:  БОК · БЕДРО · КОЛЕНО",
            fontproperties=light, fontsize=11, color=GRAY, ha="center")

    # --- силовая ветка -----------------------------------------------------
    py = fan_y - 62
    ax.plot([M, W_MM - M], [py, py], color=INK, lw=0.6, alpha=0.35)
    ax.text(M, py - 12, "ПИТАНИЕ СЕРВОПРИВОДОВ — ОТДЕЛЬНОЕ",
            fontproperties=bold, fontsize=12, color=INK, va="top")
    ax.text(M, py - 20, "2S LiPo на приводы · логика питается отдельно · общая земля",
            fontproperties=light, fontsize=10.5, color=GRAY, va="top")

    # --- три факта ---------------------------------------------------------
    fy = py - 54
    facts = [
        ("12", "сервоприводов\nQY3240MG HV"),
        ("4 × 3", "ноги и суставы\nбок · бедро · колено"),
        ("1", "контроллер, без\nвнешнего компьютера"),
    ]
    colw = (W_MM - 2 * M) / 3.0
    for i, (big, small) in enumerate(facts):
        x = M + colw * i
        ax.plot([x, x + colw - 14], [fy, fy], color=INK, lw=0.8)
        ax.text(x, fy - 20, big, fontproperties=bold, fontsize=40, color=INK,
                va="baseline")
        ax.text(x, fy - 28, small, fontproperties=light, fontsize=10.5,
                color=GRAY, va="top", linespacing=1.5)

    # --- нижний блок: пульт + QR ------------------------------------------
    by = 56.0
    ax.plot([M, W_MM - M], [by + 86, by + 86], color=INK, lw=0.6, alpha=0.35)
    ax.text(M, by + 70, "УПРАВЛЕНИЕ С ТЕЛЕФОНА", fontproperties=bold,
            fontsize=16, color=INK, va="baseline")
    ax.text(M, by + 58,
            "Подключитесь к Wi-Fi стенда, наведите камеру на код\n"
            "и нажмите кнопку. Робот двигается только под присмотром оператора.",
            fontproperties=light, fontsize=11, color=GRAY, va="top",
            linespacing=1.6)
    ax.text(M, by + 26, args.url, fontproperties=reg, fontsize=12.5,
            color=SIGNAL, va="baseline")

    if not args.no_qr:
        try:
            import segno
            qr = segno.make(args.url, error="m")
            mat = [list(row) for row in qr.matrix]
            n = len(mat)
            side = 62.0
            step = side / n
            x0, y0 = W_MM - M - side, by + 10
            ax.add_patch(patches.Rectangle((x0 - 5, y0 - 5), side + 10, side + 10,
                                           facecolor="white", edgecolor=INK, lw=0.8))
            for r, row in enumerate(mat):
                for c, v in enumerate(row):
                    if v:
                        ax.add_patch(patches.Rectangle(
                            (x0 + c * step, y0 + (n - 1 - r) * step),
                            step, step, color=INK, lw=0))
        except ImportError:
            ax.text(W_MM - M, by + 30, "(segno не установлен — QR не нарисован)",
                    fontproperties=light, fontsize=10, color=GRAY, ha="right")

    # --- подвал ------------------------------------------------------------
    ax.plot([M, W_MM - M], [38, 38], color=INK, lw=1.0)
    ax.text(M, 28, "РАМА OPENQUADRUPED · ПЕЧАТЬ FDM · ПРОШИВКА СОБСТВЕННАЯ",
            fontproperties=light, fontsize=9.5, color=GRAY, va="baseline")
    ax.text(W_MM - M, 28, "ТАШКЕНТ · 2026", fontproperties=light, fontsize=9.5,
            color=GRAY, ha="right", va="baseline")

    fig.savefig(args.out + ".pdf", facecolor=PAPER)
    fig.savefig(args.out + ".png", facecolor=PAPER, dpi=110)
    print("готово:", os.path.abspath(args.out + ".pdf"))


if __name__ == "__main__":
    main()
