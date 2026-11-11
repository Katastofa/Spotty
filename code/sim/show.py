"""
ВЫСТАВОЧНЫЙ ПОКАЗ — цифровой двойник Spotty по кругу.

    python show.py                         окно PyBullet, показ крутится сам
    python show.py --params results/X.json своя походка вместо best_robust_m100
    python show.py --video show.mp4        записать ролик (без окна, для ТВ)
    python show.py --noshadow              если окно тормозит на слабом ноутбуке

Клавиши в окне:  N — следующая сцена,  Q — выход.

ЧТО НА ЭКРАНЕ. Всё, что двигает суставы, — настоящий gait_test.ino через
hostcheck (как во всём симуляторе, R1). Питоновской копии походки нет.
Поверх нарисовано только то, что уже считает player.py: следы стоп,
центр масс, опорный многоугольник.

ПОДПИСИ В ОКНЕ — ЛАТИНИЦЕЙ: у отладочного шрифта PyBullet нет кириллицы
(см. view.py). В ролике (--video) текст рисуется PIL, там по-русски.

Сцены не утверждают ничего сверх симуляции: числа на экране — это
метрики модели, у которой массы и серво ПРИНЯТЫ, а не измерены.
"""
import argparse
import collections
import json
import math
import os
import sys
import time

import pybullet as p
import pybullet_data

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import evaluate as E, firmware, model as M
from spotty_sim.player import Sim, read_csv, support_margin

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PARAMS = os.path.join(HERE, "results", "best_robust_m100.json")
LEG_RGB = {"FR": (0.95, 0.35, 0.25), "FL": (0.20, 0.75, 0.95),
           "BR": (0.98, 0.75, 0.15), "BL": (0.55, 0.85, 0.30)}
FPS = 30
# Окно PyBullet шире по углу обзора, чем кадр ролика: камеру туда подвигаем ближе.
GUI_ZOOM = 0.62
NO_SHADOWS = False


# --------------------------------------------------------------------------
# Сцены. ru — для ролика, en — для окна PyBullet (без кириллицы).
# --------------------------------------------------------------------------
def scenes(best):
    tall = dict(best, height=180.0)
    noshift = dict(best, shiftgain=0.0)
    return [
        dict(key="title", card=True, secs=4.0,
             ru=("SPOTTY", "Четвероногий робот · ESP32-S3 · 12 сервоприводов"),
             en=("SPOTTY", "quadruped robot | ESP32-S3 | 12 servos")),
        dict(key="twin", params=best, cycles=8, cam="orbit",
             ru=("Цифровой двойник",
                 "Суставами управляет тот же код прошивки, что прошит в робота"),
             en=("DIGITAL TWIN",
                 "joints driven by the SAME firmware code as the real robot")),
        dict(key="balance", params=best, cycles=5, cam="high", support=True,
             ru=("Почему он не падает",
                 "Точка — центр масс. Пока она внутри треугольника опоры, робот устойчив"),
             en=("WHY IT DOES NOT FALL",
                 "dot = center of mass; inside the support triangle = stable")),
        dict(key="learn1", params=tall, cycles=3, cam="side", tag=1,
             ru=("Подбор походки · попытка 1",
                 "Корпус поднят слишком высоко"),
             en=("TUNING THE GAIT | try 1", "body too high")),
        dict(key="learn2", params=noshift, cycles=3, cam="side", tag=2,
             ru=("Подбор походки · попытка 2",
                 "Корпус не смещается над опорой: идёт, но медленно"),
             en=("TUNING THE GAIT | try 2", "no body shift: walks, but slowly")),
        dict(key="learn3", params=best, cycles=3, cam="side", tag=3,
             ru=("Подбор походки · итог",
                 "Параметры подобраны оптимизатором (метод перекрёстной энтропии)"),
             en=("TUNING THE GAIT | result",
                 "parameters found by an optimizer (cross-entropy method)")),
        dict(key="end", card=True, secs=4.0,
             ru=("Код → симуляция → робот",
                 "Одна и та же прошивка работает и здесь, и в железе"),
             en=("CODE -> SIMULATION -> ROBOT",
                 "one firmware runs both here and on the hardware")),
    ]


# --------------------------------------------------------------------------
class ShowSim(Sim):
    """Sim с окном нужного размера, тенями и без панелей."""

    def __init__(self, m, urdf, gui, width=1600, height=900):
        self.m = m
        self.gui = gui
        opts = "--width=%d --height=%d" % (width, height)
        self.cid = p.connect(p.GUI, options=opts) if gui else p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.cid)
        p.setGravity(0, 0, -9.81, physicsClientId=self.cid)
        self.dt = 1.0 / m.sim_hz
        p.setTimeStep(self.dt, physicsClientId=self.cid)
        p.setPhysicsEngineParameter(numSolverIterations=80, fixedTimeStep=self.dt,
                                    deterministicOverlappingPairs=1,
                                    physicsClientId=self.cid)
        self.plane = p.loadURDF("plane.urdf", physicsClientId=self.cid)
        p.changeDynamics(self.plane, -1, lateralFriction=m.ground_friction,
                         restitution=m.restitution, physicsClientId=self.cid)
        self.urdf = urdf
        self.robot = None
        if gui:
            for flag in (p.COV_ENABLE_GUI, p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
                         p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                         p.COV_ENABLE_RGB_BUFFER_PREVIEW):
                p.configureDebugVisualizer(flag, 0, physicsClientId=self.cid)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0 if NO_SHADOWS else 1,
                                       physicsClientId=self.cid)

    def feet_and_com(self):
        cid = self.cid
        pos, orn = p.getBasePositionAndOrientation(self.robot, physicsClientId=cid)
        states = p.getLinkStates(self.robot, self.all_links,
                                 computeForwardKinematics=True, physicsClientId=cid)
        T = self.m.tibia_mm * M.MM
        com = [pos[j] * self.m.body_mass_kg for j in range(3)]
        for i, ls in enumerate(states):
            for j in range(3):
                com[j] += ls[0][j] * self.link_mass[i]
        com = [c / self.total_mass for c in com]
        feet = {}
        for leg in M.LEGS:
            ls = states[self.shank_id[leg]]
            rot = p.getMatrixFromQuaternion(ls[5])
            f = [ls[4][j] + rot[3 * j + 2] * (-T) for j in range(3)]
            touch = f[2] < self.m.foot_r_mm * M.MM * 1.15
            feet[leg] = (f, touch)
        roll, pitch, _ = p.getEulerFromQuaternion(orn)
        return pos, (roll, pitch), com, feet


# --------------------------------------------------------------------------
class Markers:
    """Метки на полу — настоящая геометрия сцены, поэтому корпус их честно
    загораживает и в окне, и в ролике. Без столкновений и массы."""
    PER_LEG = 14

    def __init__(self, cid, support3d=True):
        self.cid = cid
        # В ролике (TinyRenderer) прозрачности нет, и корпус сверху закрывает
        # центр масс — там многоугольник рисуется поверх кадра «рентгеном».
        self.support3d = support3d
        self.hide = [0, 0, -5]
        # Сто с лишним тел: в окне создавать их с выключенной отрисовкой и
        # пачкой, иначе на встроенной графике это десятки секунд.
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=cid)
        self.prints = {}
        self.next = {}
        for leg in M.LEGS:
            rgba = list(LEG_RGB[leg]) + [1]
            vs = p.createVisualShape(p.GEOM_CYLINDER, radius=0.011, length=0.001,
                                     rgbaColor=rgba, physicsClientId=cid)
            self.prints[leg] = self._batch(vs, self.PER_LEG)
            self.next[leg] = 0
        # Рёбра опорного многоугольника разной длины, а визуальную форму
        # масштабировать нельзя, поэтому ребро выкладывается из коротких плиток.
        self.seg = []
        vs = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.006, 0.003, 0.0006],
                                 rgbaColor=[0.1, 0.7, 0.2, 1], physicsClientId=cid)
        self.seg_vs = vs
        self.seg = self._batch(vs, 4 * 26)
        vs = p.createVisualShape(p.GEOM_CYLINDER, radius=0.014, length=0.002,
                                 rgbaColor=[0.1, 0.7, 0.2, 1], physicsClientId=cid)
        self.com_vs = vs
        self.com = p.createMultiBody(0, -1, vs, self.hide, physicsClientId=cid)
        self.ok = None
        self.last_touch = {}
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=cid)

    def _batch(self, vs, n):
        ids = p.createMultiBody(0, -1, vs, self.hide,
                                batchPositions=[self.hide] * n, physicsClientId=self.cid)
        return list(ids) if isinstance(ids, (list, tuple)) else [ids]

    def reset(self):
        for leg in M.LEGS:
            for b in self.prints[leg]:
                p.resetBasePositionAndOrientation(b, self.hide, [0, 0, 0, 1],
                                                  physicsClientId=self.cid)
            self.next[leg] = 0
        self.support(None)
        self.last_touch = {}

    def footprints(self, feet):
        for leg, (f, touch) in feet.items():
            if touch and not self.last_touch.get(leg):
                b = self.prints[leg][self.next[leg] % self.PER_LEG]
                self.next[leg] += 1
                p.resetBasePositionAndOrientation(b, [f[0], f[1], 0.0006], [0, 0, 0, 1],
                                                  physicsClientId=self.cid)
            self.last_touch[leg] = touch

    def support(self, st):
        if not self.support3d:
            st = None
        used = 0
        if st is not None:
            ok = st["margin"] > 0
            if ok != self.ok:
                rgba = [0.1, 0.75, 0.25, 1] if ok else [0.95, 0.5, 0.05, 1]
                for b in self.seg + [self.com]:
                    p.changeVisualShape(b, -1, rgbaColor=rgba, physicsClientId=self.cid)
                self.ok = ok
            pts = [f for f, t in st["feet"].values() if t]
            if len(pts) >= 2:
                gx = sum(q[0] for q in pts) / len(pts)
                gy = sum(q[1] for q in pts) / len(pts)
                pts.sort(key=lambda q: math.atan2(q[1] - gy, q[0] - gx))
                n = len(pts) if len(pts) > 2 else 1
                for i in range(n):
                    a, c = pts[i], pts[(i + 1) % len(pts)]
                    L = math.hypot(c[0] - a[0], c[1] - a[1])
                    yaw = math.atan2(c[1] - a[1], c[0] - a[0])
                    q = p.getQuaternionFromEuler([0, 0, yaw])
                    k = max(1, int(L / 0.012))
                    for j in range(k):
                        if used >= len(self.seg):
                            break
                        u = (j + 0.5) / k
                        p.resetBasePositionAndOrientation(
                            self.seg[used], [a[0] + (c[0] - a[0]) * u,
                                             a[1] + (c[1] - a[1]) * u, 0.0012], q,
                            physicsClientId=self.cid)
                        used += 1
            c = st["com"]
            p.resetBasePositionAndOrientation(self.com, [c[0], c[1], 0.0015],
                                              [0, 0, 0, 1], physicsClientId=self.cid)
        else:
            p.resetBasePositionAndOrientation(self.com, self.hide, [0, 0, 0, 1],
                                              physicsClientId=self.cid)
        for b in self.seg[used:]:
            p.resetBasePositionAndOrientation(b, self.hide, [0, 0, 0, 1],
                                              physicsClientId=self.cid)


# --------------------------------------------------------------------------
class Camera:
    def __init__(self):
        self.tgt = None
        self.yaw = 35.0

    def update(self, mode, base, t, dt):
        if self.tgt is None:
            self.tgt = list(base)
        k = min(1.0, dt * 2.0)                      # мягкое слежение
        for j in range(3):
            self.tgt[j] += (base[j] - self.tgt[j]) * k
        tgt = [self.tgt[0], self.tgt[1], 0.12]
        if mode == "orbit":
            self.yaw = 35.0 + 9.0 * t
            return tgt, 0.85, self.yaw, -18.0
        if mode == "high":
            return tgt, 0.70, 20.0, -62.0
        return tgt, 0.95, 0.0, -12.0                 # side: вид сбоку


# --------------------------------------------------------------------------
def run_scene(sim, sc, on_frame, markers, gait=3):
    """Проиграть сцену. on_frame(state) зовётся с частотой FPS по времени модели."""
    if sc.get("card"):
        n = int(sc["secs"] * FPS)
        for i in range(n):
            if on_frame(dict(card=True, i=i, n=n)) == "next":
                return
        return
    full = dict(E.DEFAULTS)
    full.update(sc["params"])
    summ, csv = firmware.trace(full, cycles=sc["cycles"], gait=gait)
    ts, rows, settle = read_csv(csv)
    try:
        os.remove(csv)
    except OSError:
        pass
    sim.spawn(rows[0], full["height"])
    markers.reset()
    if sc.get("support") and sim.gui:
        # в окне корпус полупрозрачный — видно центр масс под ним
        p.changeVisualShape(sim.robot, -1, rgbaColor=[0.20, 0.45, 0.75, 0.35],
                            physicsClientId=sim.cid)
    lag = max(0, int(round(sim.m.latency_ms / (sim.dt * 1000.0))))
    buf = [sim._to_urdf(rows[0])] * (lag + 1)
    tick_ms = summ["tick_ms"]
    spt = max(1, int(round(tick_ms / 1000.0 / sim.dt)))
    frame_every = max(1, int(round(1000.0 / FPS / tick_ms)))
    start = None
    fell_at = None
    # после конца траектории / падения — ещё немного физики, чтобы досмотреть
    extra = int(1.5 * 1000.0 / tick_ms)
    k, total = 0, len(rows) + extra
    while k < total:
        row = rows[min(k, len(rows) - 1)]
        buf.append(sim._to_urdf(row))
        cmd = buf[-(lag + 1)]
        if len(buf) > lag + 4:
            del buf[0]
        for _ in range(spt):
            sim._drive(cmd)
            p.stepSimulation(physicsClientId=sim.cid)
        if k % frame_every == 0:
            base, (roll, pitch), com, feet = sim.feet_and_com()
            if start is None and k >= settle:
                start = base
            if fell_at is None and (abs(roll) > math.radians(55) or
                                    abs(pitch) > math.radians(55) or base[2] < 0.03):
                fell_at = k
                total = min(total, k + extra)
            dist = (base[0] - start[0]) * 1000.0 if start else 0.0
            touching = [(f[0], f[1]) for f, t in feet.values() if t]
            margin = support_margin(com[0], com[1], touching)
            st = dict(card=False, t=k * tick_ms / 1000.0, base=base, com=com,
                      feet=feet, margin=margin, dist=dist, fell=fell_at is not None,
                      gait=summ["gait"], frac=k / float(total))
            markers.footprints(feet)
            if sc.get("support"):
                markers.support(st)
            if on_frame(st) == "next":
                return
        k += 1


# --------------------------------------------------------------------------
# ОКНО
# --------------------------------------------------------------------------
def gui_loop(sim, scene_list, markers):
    cid = sim.cid
    items = {}

    def txt(key, s, pos, rgb, size):
        kw = dict(textColorRGB=rgb, textSize=size, physicsClientId=cid)
        if key in items:
            kw["replaceItemUniqueId"] = items[key]
        items[key] = p.addUserDebugText(s, pos, **kw)

    def line(key, a, b, rgb, w=3.0, life=0.0):
        kw = dict(lineColorRGB=rgb, lineWidth=w, lifeTime=life, physicsClientId=cid)
        if key and key in items:
            kw["replaceItemUniqueId"] = items[key]
        uid = p.addUserDebugLine(a, b, **kw)
        if key:
            items[key] = uid

    while True:
        for sc in scene_list:
            p.removeAllUserDebugItems(physicsClientId=cid)
            items.clear()
            cam = Camera()
            last_feet = {}
            wall = [time.time()]
            clock = {"t": 0.0}

            def on_frame(st, sc=sc, cam=cam, last_feet=last_feet, wall=wall,
                         clock=clock):
                keys = p.getKeyboardEvents(physicsClientId=cid)
                if ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED:
                    raise SystemExit
                if ord("n") in keys and keys[ord("n")] & p.KEY_WAS_TRIGGERED:
                    return "next"
                title, sub = sc["en"]
                if st["card"]:
                    base = [0, 0, 0.15] if sim.robot is None else \
                        p.getBasePositionAndOrientation(sim.robot, physicsClientId=cid)[0]
                    p.resetDebugVisualizerCamera(0.9 * GUI_ZOOM, 35 + st["i"] * 0.3, -15,
                                                 [base[0], base[1], 0.12],
                                                 physicsClientId=cid)
                    txt("t", title, [base[0], base[1], 0.42], [0.1, 0.1, 0.1], 3.0)
                    txt("s", sub, [base[0], base[1], 0.36], [0.2, 0.2, 0.2], 1.4)
                else:
                    clock["t"] += 1.0 / FPS
                    tgt, d, yaw, pitch = cam.update(sc["cam"], st["base"],
                                                    clock["t"], 1.0 / FPS)
                    p.resetDebugVisualizerCamera(d * GUI_ZOOM, yaw, pitch, tgt, physicsClientId=cid)
                    b = st["base"]
                    txt("t", title, [b[0], b[1], 0.40], [0.1, 0.1, 0.1], 2.2)
                    txt("s", sub, [b[0], b[1], 0.35], [0.25, 0.25, 0.25], 1.3)
                    hud = "%s | %+.0f mm | t %.1f s" % (st["gait"], st["dist"], st["t"])
                    if st["fell"]:
                        hud += "  | FELL"
                    txt("h", hud, [b[0], b[1], 0.31],
                        [0.8, 0.1, 0.1] if st["fell"] else [0.1, 0.35, 0.6], 1.3)
                    for leg, (f, touch) in st["feet"].items():
                        if leg in last_feet:
                            line(None, last_feet[leg], f, LEG_RGB[leg],
                                 4.0 if touch else 2.0, life=5.0)
                        last_feet[leg] = f
                # темп реального времени
                wall[0] += 1.0 / FPS
                lag = wall[0] - time.time()
                if lag > 0:
                    time.sleep(lag)
                elif lag < -0.5:
                    wall[0] = time.time()
                return None

            run_scene(sim, sc, on_frame, markers)


# --------------------------------------------------------------------------
# РОЛИК
# --------------------------------------------------------------------------
def video(sim, scene_list, out, markers, W=1280, H=720):
    import numpy as np
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw, ImageFont

    def font(sz, bold=False):
        name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
        for d in ("/usr/share/fonts/truetype/dejavu", "C:/Windows/Fonts"):
            path = os.path.join(d, name)
            if os.path.exists(path):
                return ImageFont.truetype(path, sz)
        for alt in ("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf",):
            if os.path.exists(alt):
                return ImageFont.truetype(alt, sz)
        return ImageFont.load_default()

    F_BIG, F_T, F_S, F_H = font(96, True), font(40, True), font(26), font(24, True)
    wr = imageio.get_writer(out, fps=FPS, codec="libx264", quality=8,
                            macro_block_size=1, pixelformat="yuv420p")
    cid = sim.cid

    def proj(V, P, pt):
        v = np.array([pt[0], pt[1], pt[2], 1.0])
        c = P @ (V @ v)
        if c[3] <= 1e-6:
            return None
        return ((c[0] / c[3] + 1) * 0.5 * W, (1 - c[1] / c[3]) * 0.5 * H)

    def banner(img, title, sub, alpha=1.0):
        d = ImageDraw.Draw(img, "RGBA")
        d.rectangle([0, 0, W, 118], fill=(12, 18, 28, int(200 * alpha)))
        d.text((40, 18), title, font=F_T, fill=(255, 255, 255, int(255 * alpha)))
        d.text((40, 70), sub, font=F_S, fill=(190, 210, 230, int(255 * alpha)))

    def centered(d, y, s, f, fill):
        w = d.textlength(s, font=f)
        d.text(((W - w) / 2, y), s, font=f, fill=fill)

    for sc in scene_list:
        print("сцена:", sc["key"], flush=True)
        cam = Camera()
        trails = {leg: collections.deque(maxlen=FPS * 5) for leg in M.LEGS}
        prints = collections.deque(maxlen=60)
        clock = {"t": 0.0}

        def on_frame(st, sc=sc, cam=cam, trails=trails, prints=prints, clock=clock):
            title, sub = sc["ru"]
            if st["card"]:
                img = Image.new("RGB", (W, H), (12, 18, 28))
                d = ImageDraw.Draw(img)
                a = min(1.0, st["i"] / 12.0, (st["n"] - st["i"]) / 12.0)
                c = lambda r, g, b: (int(12 + (r - 12) * a), int(18 + (g - 18) * a),
                                     int(28 + (b - 28) * a))
                if sc["key"] == "title":
                    centered(d, H / 2 - 110, title, F_BIG, c(255, 255, 255))
                    centered(d, H / 2 + 20, sub, F_S, c(150, 200, 240))
                else:
                    centered(d, H / 2 - 60, title, F_T, c(255, 255, 255))
                    centered(d, H / 2 + 5, sub, F_S, c(150, 200, 240))
                wr.append_data(np.asarray(img))
                return None
            clock["t"] += 1.0 / FPS
            tgt, dist, yaw, pitch = cam.update(sc["cam"], st["base"], clock["t"],
                                               1.0 / FPS)
            Vf = p.computeViewMatrixFromYawPitchRoll(tgt, dist, yaw, pitch, 0, 2)
            Pf = p.computeProjectionMatrixFOV(45, W / float(H), 0.01, 20)
            _, _, rgb, _, _ = p.getCameraImage(
                W, H, Vf, Pf, renderer=p.ER_TINY_RENDERER, shadow=1,
                lightDirection=[0.4, -0.6, 1.0], physicsClientId=cid)
            img = Image.fromarray(np.reshape(np.array(rgb, dtype=np.uint8),
                                             (H, W, 4))[:, :, :3])
            V = np.array(Vf).reshape(4, 4).T
            P = np.array(Pf).reshape(4, 4).T
            d = ImageDraw.Draw(img, "RGBA")
            if sc.get("support"):
                ok = st["margin"] > 0
                col = (40, 200, 90) if ok else (245, 140, 20)
                pts = [f for f, t in st["feet"].values() if t]
                if len(pts) >= 2:
                    gx = sum(q[0] for q in pts) / len(pts)
                    gy = sum(q[1] for q in pts) / len(pts)
                    pts.sort(key=lambda q: math.atan2(q[1] - gy, q[0] - gx))
                    poly = [proj(V, P, [q[0], q[1], 0.002]) for q in pts]
                    if all(poly):
                        if len(poly) > 2:
                            d.polygon(poly, fill=col + (60,))
                        d.line(poly + [poly[0]], fill=col + (255,), width=5)
                c = st["com"]
                q = proj(V, P, [c[0], c[1], 0.002])
                if q:
                    d.ellipse([q[0] - 11, q[1] - 11, q[0] + 11, q[1] + 11],
                              fill=col + (255,), outline=(255, 255, 255, 255), width=3)
                    lbl = "центр масс внутри опоры" if ok else "опора на диагональ"
                    tw = d.textlength(lbl, font=F_H)
                    d.rounded_rectangle([q[0] + 18, q[1] - 16, q[0] + 30 + tw, q[1] + 18],
                                        8, fill=(12, 18, 28, 190))
                    d.text((q[0] + 24, q[1] - 14), lbl, font=F_H, fill=col + (255,))
            a = min(1.0, clock["t"] * 2.0)
            banner(img, title, sub, a)
            hud = "пройдено %+.0f мм · %.1f с" % (st["dist"], st["t"])
            d = ImageDraw.Draw(img, "RGBA")
            d.rectangle([0, H - 56, W, H], fill=(12, 18, 28, 170))
            d.text((40, H - 44), hud, font=F_H, fill=(255, 255, 255))
            if sc.get("tag"):
                d.text((W - 300, H - 44), "PyBullet · прошивка gait_test.ino",
                       font=font(18), fill=(160, 180, 200))
            if st["fell"]:
                d.rounded_rectangle([W / 2 - 150, H / 2 - 50, W / 2 + 150, H / 2 + 50],
                                    18, fill=(200, 40, 40, 220))
                centered(d, H / 2 - 28, "УПАЛ", F_T, (255, 255, 255))
            wr.append_data(np.asarray(img))
            return None

        run_scene(sim, sc, on_frame, markers)
    wr.close()
    print("готово:", out)


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", default=DEFAULT_PARAMS)
    ap.add_argument("--video", default=None, help="записать mp4 вместо окна")
    ap.add_argument("--only", default=None, help="сцены через запятую (отладка)")
    ap.add_argument("--noshadow", action="store_true",
                    help="без теней — если на слабом ноутбуке окно тормозит")
    args = ap.parse_args()
    global NO_SHADOWS
    NO_SHADOWS = args.noshadow
    with open(args.params, "r", encoding="utf-8") as f:
        blob = json.load(f)
    best = dict(blob.get("params", blob))
    scene_list = scenes(best)
    if args.only:
        keep = args.only.split(",")
        scene_list = [s for s in scene_list if s["key"] in keep]

    m, urdf, geom = E.prepare_urdf(HERE)
    sim = ShowSim(m, urdf, gui=args.video is None)
    markers = Markers(sim.cid, support3d=args.video is None)
    try:
        if args.video:
            video(sim, scene_list, args.video, markers)
        else:
            print("Показ идёт по кругу.  N — следующая сцена,  Q — выход.")
            gui_loop(sim, scene_list, markers)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        sim.close()


if __name__ == "__main__":
    main()
