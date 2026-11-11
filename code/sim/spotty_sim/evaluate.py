"""
Одна оценка: параметры походки -> прошивка -> физика -> число.

Фитнес собран так, чтобы его можно было прочитать глазами и поспорить с ним:
каждое слагаемое подписано и имеет вес в конфиге. Менять веса — это и есть
«сказать оптимизатору, какая походка считается хорошей».
"""
import os
import tempfile
from dataclasses import dataclass

from . import firmware, model as M

LEGS_ORDER = M.LEGS
LEG_FWD_ = M.LEG_FWD
LEG_LEFT_ = M.LEG_LEFT
from .player import Sim, read_csv

# Границы поиска. Это ПРИНЯТЫЕ рамки здравого смысла, а не пределы железа:
# пределы железа стоят в прошивке и всё равно обрежут лишнее.
SPACE = {
    "span":   (12.0, 55.0),    # полуразмах шага, мм
    "lift":   (5.0, 32.0),     # подъём стопы в переносе, мм
    "vel":    (25.0, 140.0),   # заказанная скорость корпуса, мм/с
    "swing":  (250.0, 1200.0),  # время переноса, мс
    "height": (125.0, 205.0),  # глубина стопы под осью хипа, мм
    "shiftx": (-30.0, 30.0),   # сдвиг корпуса вперёд, мм
}

# ВНИМАНИЕ. pTrimDeg в декартовом движке не участвует: gaitPoseCartesian его
# не читает. Прошивка это знает и команду 'b' в декартовом режиме отклоняет,
# предлагая 'X'. Стенд ставит переменные напрямую, мимо разбора команд, так
# что trim здесь просто ничего не менял бы. Продольный аналог — pShiftXMm,
# он же shiftx (команда 'X').

# Пространство ФАЗОВОГО СДВИГА КОРПУСА.
#
# Искать восемь независимых узлов вслепую — расточительно: нужный сдвиг
# считается из геометрии опорного треугольника (см. analytic_pattern ниже).
# Поэтому ищется не сам сдвиг, а ЧТО С НИМ СДЕЛАТЬ: во сколько раз взять,
# насколько опередить по фазе и какой постоянный перекос добавить. Четыре
# числа вместо шестнадцати, и каждое имеет смысл, который можно обсуждать.
SHIFT_SPACE = {
    "gain_x": (0.0, 1.5),      # доля от геометрически нужного сдвига ВПЕРЁД
    "gain_y": (0.0, 2.5),      # доля от геометрически нужного сдвига ВБОК
    "lead":   (-0.25, 0.25),   # опережение по фазе, доли цикла
    "bias_x": (-30.0, 30.0),   # постоянный сдвиг корпуса вперёд, мм
    "bias_y": (-20.0, 20.0),   # постоянный сдвиг корпуса вбок, мм
}

# Пространство УСТОЙЧИВОГО поиска (mode="robust").
#
# Отличий от SPACE три, и каждое — вывод, а не вкус:
#
#   height  сверху 158, а не 205. Свип по высоте (results/log_height.txt)
#           показал обрыв между 150 и 160 мм, который НЕ сдвигается ничем:
#           ни походкой, ни переносом корпуса, ни моментом серво вдвое, ни
#           жёсткостью контура, ни нулевой латентностью, ни снятым
#           ограничителем скорости. Пускать поиск выше — жечь время на
#           области, где всё падает.
#   shiftgain  добавлен: перенос корпуса из прошивки теперь параметр, а не
#           решение задним числом. Пусть поиск сам скажет, сколько его надо.
#   maxrate НЕ входит: это не параметр походки, а предохранитель. Он
#           объявляется снаружи и печатается в отчёте.
ROBUST_SPACE = {
    "span":      (12.0, 55.0),
    "lift":      (5.0, 32.0),
    "vel":       (25.0, 140.0),
    "swing":     (250.0, 1200.0),
    "height":    (125.0, 158.0),
    "shiftgain": (0.0, 1.5),
}

NODES = 8                      # узлов таблицы сдвига на цикл


def _cyc(tab, u):
    """Циклическая линейная интерполяция таблицы по фазе 0..1."""
    n = len(tab)
    u = u - int(u) + (1.0 if u < 0 else 0.0)
    f = u * n
    i0 = min(int(f), n - 1)
    return tab[i0] + (tab[(i0 + 1) % n] - tab[i0]) * (f - i0)


def decode(mode, x, fixed=None):
    """Вектор оптимизатора -> параметры прошивки."""
    out = {k: v for k, v in (fixed or {}).items() if not k.startswith("_")}
    if mode == "shift":
        v = dict(zip(sorted(SHIFT_SPACE), x))
        px, py = fixed["_px"], fixed["_py"]
        sx, sy = [], []
        for j in range(NODES):
            u = j / float(NODES) + v["lead"]
            sx.append(v["gain_x"] * _cyc(px, u) + v["bias_x"])
            sy.append(v["gain_y"] * _cyc(py, u) + v["bias_y"])
        out["sx"], out["sy"] = sx, sy
    else:
        out.update(dict(zip(sorted(space_of(mode)), x)))
    return out


def analytic_pattern(m, gait_phase, duty, margin_target=None):
    """Сдвиг, который ТРЕБУЕТ ГЕОМЕТРИЯ, а не подбор.

    Пока одна нога в переносе, опора — треугольник из трёх оставшихся стоп.
    У четвероногого с ЦТ в середине рамы диагональ этого треугольника проходит
    ровно через центр, и запас устойчивости РАВЕН НУЛЮ. Точка максимального
    запаса внутри треугольника — его инцентр; сдвиг корпуса в инцентр и есть
    то, что должно быть в таблице.

    Возвращает две таблицы по NODES узлов: сдвиг вперёд и сдвиг вбок, мм.
    """
    from .player import support_margin
    half_l = m.base_long_mm / 2.0
    half_w = m.base_lat_mm / 2.0 + m.hip_mm
    foot = {}
    for i, leg in enumerate(LEGS_ORDER):
        foot[leg] = (LEG_FWD_[i] * half_l, LEG_LEFT_[i] * half_w)
    # когда какая нога уходит в перенос: глобальная фаза (duty - фаза ноги)
    lift_at = {}
    for i, leg in enumerate(LEGS_ORDER):
        lift_at[leg] = (duty - gait_phase[i]) % 1.0

    px = [0.0] * NODES
    py = [0.0] * NODES
    for j in range(NODES):
        u = (j + 0.5) / float(NODES)          # середина интервала узла
        # чья это фаза переноса
        swinging = None
        for leg, t0 in lift_at.items():
            d = (u - t0) % 1.0
            if d < (1.0 - duty):
                swinging = leg
        if swinging is None:                   # все четыре в опоре — не двигаемся
            continue
        tri = [foot[l] for l in LEGS_ORDER if l != swinging]
        px[j], py[j] = _incenter(tri)
    return px, py


def _incenter(tri):
    """Центр вписанной окружности — точка максимального запаса в треугольнике."""
    import math
    a = math.dist(tri[1], tri[2])
    b = math.dist(tri[0], tri[2])
    c = math.dist(tri[0], tri[1])
    s = a + b + c
    return ((a * tri[0][0] + b * tri[1][0] + c * tri[2][0]) / s,
            (a * tri[0][1] + b * tri[1][1] + c * tri[2][1]) / s)


def space_of(mode):
    if mode == "shift":
        return SHIFT_SPACE
    if mode == "robust":
        return ROBUST_SPACE
    return SPACE


def swing_for_duty(span, vel, duty):
    """duty и период в декартовом движке ВЫВОДЯТСЯ из span, vel и swing.
    Хотите задать duty — вот обратная формула для swing."""
    t_stance = 2.0 * span / vel
    return 1000.0 * t_stance * (1.0 - duty) / duty

DEFAULTS = {"span": 30.0, "lift": 15.0, "vel": 60.0, "swing": 900.0,
            "height": 190.0, "trim": 17.0}


@dataclass
class Weights:
    """Что считается хорошей походкой. Меняйте — меняется результат обучения."""
    w_speed: float = 3.0        # за пройденный путь вперёд, на метр
    w_drift: float = 1.5        # штраф за боковой снос, на метр
    w_yaw: float = 0.004        # штраф за увод курса, на градус
    w_roll: float = 0.010       # штраф за качку, на градус СКЗ
    w_pitch: float = 0.006      # штраф за клевок, на градус СКЗ
    w_slip: float = 0.0002      # штраф за проскальзывание стоп, на мм
    w_energy: float = 0.004     # штраф за работу приводов, на джоуль
    w_track: float = 0.02       # штраф за отставание сустава от команды, на град
    w_margin: float = 0.0       # ЗА запас устойчивости, на мм среднего запаса
    w_margin_neg: float = 0.0   # штраф за долю времени с отрицательным запасом
    fall_penalty: float = 5.0   # падение — не «чуть хуже», а отдельный класс


_SIM = {}


def _sim(m: M.Model, urdf_path, gui=False):
    key = (id(m), urdf_path, gui)
    if key not in _SIM:
        _SIM[key] = Sim(m, urdf_path, gui=gui)
    return _SIM[key]


def prepare_urdf(out_dir=None, base_long_mm=None, gait=0):
    """Собрать URDF из геометрии прошивки. Возвращает (модель, путь)."""
    geom = firmware.geometry(gait)
    m = M.Model().with_geometry(geom["geom_mm"])
    if base_long_mm:
        m.base_long_mm = float(base_long_mm)
    out_dir = out_dir or os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(out_dir, "spotty_gen.urdf")   # generated, not committed by hand
    with open(path, "w", encoding="utf-8") as f:
        f.write(M.build_urdf(m, limits_deg=_limits(geom)))
    return m, path, geom


def _limits(geom):
    L = geom["limits_deg"]
    return [[L[3 * i], L[3 * i + 1], L[3 * i + 2]] for i in range(4)]


def evaluate(params, m, urdf_path, cycles=4.0, gait=0, gui=False, realtime=False,
             weights=None, keep_csv=None):
    w = weights or Weights()
    full = dict(DEFAULTS)
    full.update(params)
    # ОДНА продольная база на прогон. URDF строится из m.base_long_mm, а
    # прошивка считает из неё амплитуду переноса корпуса — разойтись им нельзя
    # ни на миллиметр.
    #
    # В норме подавать нечего: база пришла ИЗ прошивки, туда же и вернётся.
    # Подаём только когда её подменили в модели (свип по чувствительности,
    # или прошивка без замера M4) — и тогда сводка стенда пометит прогон как
    # опирающийся на подставленное число.
    if abs(m.base_long_mm - m.base_long_fw_mm) > 1e-6:
        full.setdefault("baselong", m.base_long_mm)
    summary, csv = firmware.trace(full, out_csv=keep_csv, cycles=cycles, gait=gait)
    ts, rows, settle = read_csv(csv)
    sim = _sim(m, urdf_path, gui=gui)
    met = sim.run(ts, rows, settle, full["height"], tick_ms=summary["tick_ms"],
                  realtime=realtime)
    if keep_csv is None:
        try:
            os.remove(csv)
        except OSError:
            pass

    if met["fell"]:
        # УПАЛ — значит остальные метрики описывают, как именно он лежал.
        # Складывать их бессмысленно: оптимизатор мгновенно выучивает «ехать
        # далеко, падая». Содержательно здесь только одно — сколько он
        # продержался. Это и есть градиент, ведущий обратно к «устоял».
        score = -w.fall_penalty + 2.0 * met.get("survive_frac", 0.0)
        if summary.get("ik_fail") or summary.get("pose_ik_fail"):
            score -= 1.0
        return score, met, summary, full

    score = (w.w_speed * met["x_m"]
             - w.w_drift * abs(met["y_m"])
             - w.w_yaw * abs(met["yaw_deg"])
             - w.w_roll * met["roll_rms_deg"]
             - w.w_pitch * met["pitch_rms_deg"]
             - w.w_slip * met["slip_mm"]
             - w.w_energy * met["energy_j"]
             - w.w_track * met["track_deg"]
             + w.w_margin * max(-50.0, met["margin_mean_mm"])
             - w.w_margin_neg * met["margin_neg_frac"])
    if summary.get("ik_fail") or summary.get("pose_ik_fail"):
        score -= w.fall_penalty          # отказ IK — не походка вовсе
    # Прошивка не пустит на железо цикл, который просит от сустава больше
    # разрешённого. Оптимизатору об этом лучше знать заранее.
    over = summary["peak_rate_dps"] - summary["max_rate_dps"]
    if over > 0:
        score -= 0.02 * over
    return score, met, summary, full
