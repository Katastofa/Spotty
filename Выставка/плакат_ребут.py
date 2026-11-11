# -*- coding: utf-8 -*-
"""
Плакат A2 «Ребут» для выставочного стенда. Стиль — тот же, что у плакат.py
(см. ФИЛОСОФИЯ_ПЛАКАТА.md): тёплая бумага, графит, один сигнальный синий.
Идея листа: биография робота в виде журнала загрузки из монитора порта —
каждая поломка строкой [ERR], под ней починка, в конце [ OK ].

    python плакат_ребут.py                               адрес пульта по умолчанию
    python плакат_ребут.py --url http://192.168.1.50:8080/
    python плакат_ребут.py --no-qr

Нужны: matplotlib, segno (pip install matplotlib segno).
Результат: ПЛАКАТ_РЕБУТ_A2.pdf (вектор, для печати) и .png (посмотреть).
"""
import argparse
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
ERR = "#c4462f"
WARN = "#c7921c"
LEG = {"FR": "#d2563f", "FL": "#3499c4", "BR": "#d9a227", "BL": "#6a9e4b"}

# Журнал. Только то, что было на самом деле (см. историю в gait_test.ino).
LOG = [
    ("ok",   "ESP32-S3 · PCA9685 · 12 × QY3240MG HV · 2S LiPo", None),
    ("err",  "первый запуск: ноги вывернуло в разные стороны",
             "генератор драйвера 27 МГц, а не 25 — импульсы шли на 7 % короче"),
    ("err",  "выгорел чип: земля отключена под питанием",
             "6 часов выпаивания, новый модуль — сломан и тоже починен"),
    ("err",  "при включении подпрыгивает всеми четырьмя лапами",
             "старт из позы лёжа и плавный подъём без рывка"),
    ("err",  "драйвер молчит: делитель = 254 (0xFE)",
             "выпал провод SDA — теперь прошивка сообщает об этом сама"),
    ("warn", "лёгкий занос влево",
             "не баг, а характер"),
    ("ok",   "ШИМ подан. Поза достигнута. Идёт.", None),
]
TAG = {"ok": ("[  OK  ]", SIGNAL), "err": ("[ ERR  ]", ERR), "warn": ("[ WARN ]", WARN)}


def font(weight="normal", stretch="normal", family="DejaVu Sans"):
    fp = FontProperties(family=family, weight=weight, stretch=stretch)
    findfont(fp, fallback_to_default=False)
    return fp


def rounded(ax, x, y, w, h, r=3, **kw):
    ax.add_patch(patches.FancyBboxPatch(
        (x + r, y + r), w - 2 * r, h - 2 * r,
        boxstyle="round,pad=%f" % r, linewidth=kw.pop("lw", 1.2), **kw))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.137.1:8080/")
    ap.add_argument("--no-qr", action="store_true")
    ap.add_argument("--out", default="ПЛАКАТ_РЕБУТ_A2")
    args = ap.parse_args()

    reg, bold, light = font(), font("bold"), font(stretch="condensed")
    mono, monob = font(family="DejaVu Sans Mono"), font("bold", family="DejaVu Sans Mono")

    fig = plt.figure(figsize=(W_MM * MM, H_MM * MM))
    fig.patch.set_facecolor(PAPER)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W_MM)
    ax.set_ylim(0, H_MM)
    ax.axis("off")

    M = 30.0

    # --- шапка -------------------------------------------------------------
    ax.text(M - 2, H_MM - 92, "РЕБУТ", fontproperties=bold, fontsize=150,
            color=INK, va="baseline")
    ax.text(M, H_MM - 108, "ЧЕТВЕРОНОГИЙ  ШАГАЮЩИЙ  РОБОТ  ·  ПРОЕКТ SPOTTY",
            fontproperties=light, fontsize=15, color=GRAY, va="baseline")
    ax.text(W_MM - M, H_MM - 108, "rst:0x1 (POWERON)", fontproperties=mono,
            fontsize=11, color=GRAY, ha="right", va="baseline")
    ax.plot([M, W_MM - M], [H_MM - 118, H_MM - 118], color=INK, lw=1.0)

    ax.text(M, H_MM - 150, "Сгорел. Перепаялся. Пошёл.", fontproperties=bold,
            fontsize=38, color=INK, va="baseline")

    # --- журнал загрузки ---------------------------------------------------
    top = H_MM - 172
    row, sub = 12.5, 9.0
    height = 30 + sum(row + (sub if f else 0) + 3 for _, _, f in LOG)
    bx, bw = M, W_MM - 2 * M
    rounded(ax, bx, top - height, bw, height, r=3, facecolor="#ebe7dd",
            edgecolor=INK, lw=1.0)
    ax.text(bx + 10, top - 10, "ЖУРНАЛ ЗАГРУЗКИ  ·  COM4  ·  115200",
            fontproperties=light, fontsize=10.5, color=GRAY, va="top")
    ax.text(bx + bw - 10, top - 10, "ESP32-S3", fontproperties=light,
            fontsize=10.5, color=GRAY, ha="right", va="top")

    y = top - 26
    tx = bx + 12
    mx = bx + 52
    for kind, what, fix in LOG:
        tag, col = TAG[kind]
        ax.text(tx, y, tag, fontproperties=monob, fontsize=15, color=col,
                va="baseline")
        ax.text(mx, y, what, fontproperties=monob if kind == "ok" else mono,
                fontsize=15, color=INK, va="baseline")
        y -= row
        if fix:
            ax.text(mx, y + 2.5, "→ " + fix, fontproperties=mono, fontsize=12.5,
                    color=GRAY, va="baseline")
            y -= sub
        y -= 3
    # мигающий курсор в конце журнала
    ax.add_patch(patches.Rectangle((mx, y + 4.5), 3.2, 6.5, color=SIGNAL, lw=0))

    # --- три факта ---------------------------------------------------------
    fy = top - height - 24
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
    ax.text(M, by + 70, "ПОГУЛЯЙТЕ С РЕБУТОМ", fontproperties=bold,
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
            mat = [list(r) for r in qr.matrix]
            n = len(mat)
            side = 62.0
            step = side / n
            x0, y0 = W_MM - M - side - 5, by + 10
            ax.add_patch(patches.Rectangle((x0 - 5, y0 - 5), side + 10, side + 10,
                                           facecolor="white", edgecolor=INK, lw=0.8))
            for r, rr in enumerate(mat):
                for c, v in enumerate(rr):
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
