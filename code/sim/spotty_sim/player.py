"""
Проигрыватель траектории в PyBullet + метрики.

PyBullet НЕ ЗНАЕТ О ПОХОДКЕ. Он получает 12 углов на каждом тике прошивки и
отвечает только на вопросы физики: куда уехал корпус, на сколько качнуло,
проскальзывают ли стопы, хватает ли момента.

МОДЕЛЬ ПРИВОДА (SIMULATION.md R4). POSITION_CONTROL в Bullet — это решатель
связей, он навязывает нулевую ошибку в конце шага и врёт про момент. Поэтому
здесь честный момент: tau = kp*(cmd - q) - kd*q', ограниченный характеристикой
момент-скорость tau_max = tau_stall * (1 - |q'|/w_xx), то есть на скорости
серво слабеет, как настоящее.

ЛАТЕНТНОСТЬ (R5). Команда доезжает до сустава не мгновенно: кольцевой буфер
задерживает её на latency_ms. Ноль по умолчанию — это ошибка sim-to-real №1
по arXiv 1804.10332, поэтому нуля здесь нет.
"""
import math

import pybullet as p
import pybullet_data

from . import model as M


def read_csv(path):
    """CSV прошивки -> (времена, углы[tick][12], сколько строк стоянки)."""
    ts, rows, phases = [], [], []
    with open(path, "r", encoding="utf-8") as f:
        f.readline()
        for line in f:
            v = [float(x) for x in line.split(",")]
            ts.append(v[0])
            rows.append(v[1:13])
            phases.append(v[13])
    settle = sum(1 for ph in phases if ph < 0)
    return ts, rows, settle


class Sim:
    def __init__(self, m: M.Model, urdf_path, gui=False, limits_deg=None):
        self.m = m
        self.gui = gui
        self.cid = p.connect(p.GUI if gui else p.DIRECT)
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
        self.urdf = urdf_path
        self.robot = None
        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.cid)
            p.resetDebugVisualizerCamera(1.1, 50, -20, [0, 0, 0.18],
                                         physicsClientId=self.cid)

    def close(self):
        if p.isConnected(self.cid):
            p.disconnect(self.cid)

    # --- постановка робота --------------------------------------------------
    def spawn(self, first_row, height_mm):
        if self.robot is not None:
            p.removeBody(self.robot, physicsClientId=self.cid)
        z0 = (height_mm + self.m.foot_r_mm) * M.MM + 0.001
        self.robot = p.loadURDF(self.urdf, basePosition=[0, 0, z0],
                                useFixedBase=False, physicsClientId=self.cid)
        self.jids, self.shank = [], []
        names = {}
        for i in range(p.getNumJoints(self.robot, physicsClientId=self.cid)):
            info = p.getJointInfo(self.robot, i, physicsClientId=self.cid)
            names[info[1].decode()] = i
            if info[12].decode().startswith("shank_"):
                self.shank.append((info[12].decode()[6:], i))
        for leg in M.LEGS:
            for j in ("j_hip_", "j_thigh_", "j_knee_"):
                self.jids.append(names[j + leg])
        self.shank_id = {leg: i for leg, i in self.shank}
        # массы звеньев в порядке индексов PyBullet — для настоящего ЦТ
        self.link_mass = []
        for i in range(p.getNumJoints(self.robot, physicsClientId=self.cid)):
            nm = p.getJointInfo(self.robot, i, physicsClientId=self.cid)[12].decode()
            self.link_mass.append(self.m.hip_mass_kg if nm.startswith("hip_") else
                                  self.m.thigh_mass_kg if nm.startswith("thigh_") else
                                  self.m.shank_mass_kg)
        self.all_links = list(range(len(self.link_mass)))
        self.total_mass = self.m.body_mass_kg + sum(self.link_mass)

        for i in self.jids:
            p.changeDynamics(self.robot, i, linearDamping=0.02, angularDamping=0.02,
                             jointDamping=0.0, physicsClientId=self.cid)
            p.setJointMotorControl2(self.robot, i, p.VELOCITY_CONTROL, force=0,
                                    physicsClientId=self.cid)
        for leg in M.LEGS:
            p.changeDynamics(self.robot, self.shank_id[leg],
                             lateralFriction=self.m.foot_friction,
                             spinningFriction=0.02, rollingFriction=0.001,
                             restitution=self.m.restitution, physicsClientId=self.cid)
        # ноги сразу в первой позе траектории, чтобы не падать с высоты
        for k, q in enumerate(self._to_urdf(first_row)):
            p.resetJointState(self.robot, self.jids[k], q, physicsClientId=self.cid)

    # Ошибки нулей суставов, градусы, в порядке FR/FL/BR/BL × хип/бедро/колено.
    # Это ровно то, что даёт M9: прошивка думает, что выдала угол, а качалка
    # сидит на шлице со смещением. В модели это сдвиг КОМАНДЫ, а не траектории.
    zero_err = [0.0] * 12

    def _to_urdf(self, row12):
        out = []
        for i in range(4):
            out.extend(M.to_urdf_angles(
                i, row12[3 * i] + self.zero_err[3 * i],
                row12[3 * i + 1] + self.zero_err[3 * i + 1],
                row12[3 * i + 2] + self.zero_err[3 * i + 2]))
        return out

    # --- один шаг привода ---------------------------------------------------
    def _drive(self, cmd):
        st = p.getJointStates(self.robot, self.jids, physicsClientId=self.cid)
        tau_stall = self.m.stall_torque_nm
        w_max = math.radians(self.m.no_load_dps)
        kp, kd = self.m.servo_kp, self.m.servo_kd
        taus, err = [], 0.0
        for i, (q, qd, _, _) in enumerate(st):
            e = cmd[i] - q
            err += abs(e)
            t = kp * e - kd * qd
            lim = tau_stall * max(0.0, 1.0 - abs(qd) / w_max)   # момент-скорость
            t = max(-lim, min(lim, t))
            taus.append(t)
        p.setJointMotorControlArray(self.robot, self.jids, p.TORQUE_CONTROL,
                                    forces=taus, physicsClientId=self.cid)
        return taus, st, err / len(st)

    # --- прогон -------------------------------------------------------------
    def run(self, ts, rows, settle_rows, height_mm, tick_ms=3.0, realtime=False,
            record=False):
        import time
        self.spawn(rows[0], height_mm)
        lag = max(0, int(round(self.m.latency_ms / (self.dt * 1000.0))))
        buf = [self._to_urdf(rows[0])] * (lag + 1)

        steps_per_tick = max(1, int(round(tick_ms / 1000.0 / self.dt)))
        done = 0
        metrics = _Metrics(self, settle_rows * steps_per_tick)
        metrics.record = record

        for k, row in enumerate(rows):
            buf.append(self._to_urdf(row))
            cmd = buf[-(lag + 1)]
            if len(buf) > lag + 4:
                del buf[0]
            for _ in range(steps_per_tick):
                taus, st, err = self._drive(cmd)
                p.stepSimulation(physicsClientId=self.cid)
                metrics.accumulate(taus, st, err)
                if realtime:
                    time.sleep(self.dt)
            # Метрики снимаются РАЗ В ТИК прошивки, а не раз в шаг решателя:
            # иначе сумма проскальзывания набирается из дрожания решателя,
            # а не из того, что стопа действительно поехала.
            metrics.sample(k >= settle_rows, t=ts[k])
            done = k + 1
            if metrics.fell:
                break
        metrics.finish(len(rows), settle_rows, tick_ms, done)
        return metrics.out


class _Metrics:
    def __init__(self, sim, settle_steps):
        self.s = sim
        self.fell = False
        self.n = 0
        self.energy = 0.0
        self.track = 0.0
        self.track_n = 0
        self.roll_max = self.pitch_max = 0.0
        self.roll_sq = self.pitch_sq = 0.0
        self.slip = 0.0
        self.contact_steps = 0
        self.start = None
        self.touchdown = {}
        self.z_min = 9.0
        self.record = False
        self.margin_min = 9e9
        self.margin_sum = 0.0
        self.margin_n = 0
        self.margin_neg = 0
        self.series = {"t": [], "x": [], "y": [], "z": [], "roll": [], "pitch": [],
                       "margin": []}

    def accumulate(self, taus, st, err):
        """Работа приводов и отставание — по каждому шагу решателя."""
        for (q, qd, _, _), t in zip(st, taus):
            self.energy += abs(t * qd) * self.s.dt
        self.track += err
        self.track_n += 1

    def sample(self, walking, t=0.0):
        cid = self.s.cid
        pos, orn = p.getBasePositionAndOrientation(self.s.robot, physicsClientId=cid)
        roll, pitch, yaw = p.getEulerFromQuaternion(orn)
        if self.record:
            self.series["t"].append(t)
            self.series["x"].append(pos[0])
            self.series["y"].append(pos[1])
            self.series["z"].append(pos[2])
            self.series["roll"].append(math.degrees(roll))
            self.series["pitch"].append(math.degrees(pitch))
        if walking and self.start is None:
            self.start = (pos, yaw)
        if pos[2] < self.z_min:
            self.z_min = pos[2]
        if abs(roll) > math.radians(55) or abs(pitch) > math.radians(55) or pos[2] < 0.03:
            self.fell = True
        if not walking:
            return
        self.n += 1
        self.roll_max = max(self.roll_max, abs(roll))
        self.pitch_max = max(self.pitch_max, abs(pitch))
        self.roll_sq += roll * roll
        self.pitch_sq += pitch * pitch
        # Проскальзывание считается ЗА ОПОРУ ЦЕЛИКОМ: где стопа встала и где
        # оторвалась. Сумма мелких сдвигов за шаг решателя мерила бы дрожание.
        T = self.s.m.tibia_mm * M.MM
        states = p.getLinkStates(self.s.robot, self.s.all_links,
                                 computeForwardKinematics=True, physicsClientId=cid)
        com = [pos[j] * self.s.m.body_mass_kg for j in range(3)]
        feet = []
        for i, ls in enumerate(states):
            for j in range(3):
                com[j] += ls[0][j] * self.s.link_mass[i]
        com = [c / self.s.total_mass for c in com]
        for leg in M.LEGS:
            ls = states[self.s.shank_id[leg]]
            rot = p.getMatrixFromQuaternion(ls[5])
            f = [ls[4][j] + rot[3 * j + 2] * (-T) for j in range(3)]
            touch = f[2] < self.s.m.foot_r_mm * M.MM * 1.15
            if touch:
                feet.append((f[0], f[1]))
            if touch and leg not in self.touchdown:
                self.touchdown[leg] = f
            elif not touch and leg in self.touchdown:
                a = self.touchdown.pop(leg)
                self.slip += math.hypot(f[0] - a[0], f[1] - a[1])
                self.contact_steps += 1
        margin = support_margin(com[0], com[1], feet)
        self.margin_min = min(self.margin_min, margin)
        self.margin_sum += margin
        self.margin_n += 1
        if margin < 0:
            self.margin_neg += 1
        if self.record:
            self.series["margin"].append(margin * 1000.0)

    def finish(self, rows, settle_rows, tick_ms, done=None):
        cid = self.s.cid
        pos, orn = p.getBasePositionAndOrientation(self.s.robot, physicsClientId=cid)
        roll, pitch, yaw = p.getEulerFromQuaternion(orn)
        walk_s = max(1e-6, (rows - settle_rows) * tick_ms / 1000.0)
        x0, y0, yaw0 = (self.start[0][0], self.start[0][1], self.start[1]) \
            if self.start else (0.0, 0.0, 0.0)
        n = max(1, self.n)
        self.out = {
            "fell": self.fell,
            "survive_frac": (float(done) / max(1, rows)) if done else 1.0,
            "walk_s": walk_s,
            "x_m": pos[0] - x0,
            "y_m": pos[1] - y0,
            "yaw_deg": math.degrees(_wrap(yaw - yaw0)),
            "speed_mm_s": (pos[0] - x0) * 1000.0 / walk_s,
            "roll_max_deg": math.degrees(self.roll_max),
            "pitch_max_deg": math.degrees(self.pitch_max),
            "roll_rms_deg": math.degrees(math.sqrt(self.roll_sq / n)),
            "pitch_rms_deg": math.degrees(math.sqrt(self.pitch_sq / n)),
            "slip_mm": self.slip * 1000.0,
            "energy_j": self.energy,
            "track_deg": math.degrees(self.track / max(1, self.track_n)),
            "z_min_m": self.z_min,
            "margin_min_mm": (self.margin_min * 1000.0
                              if self.margin_n else float("nan")),
            "margin_mean_mm": (self.margin_sum / max(1, self.margin_n) * 1000.0),
            "margin_neg_frac": self.margin_neg / max(1, self.margin_n),
        }
        if self.record:
            self.out["series"] = self.series


def support_margin(cx, cy, feet):
    """Знаковое расстояние от проекции ЦТ до границы опорного многоугольника.

    + внутри, − снаружи. Три и больше стоп — настоящий многоугольник; две —
    отрезок (у рыси иначе и не бывает, и запас там ВСЕГДА неположительный,
    это свойство походки, а не ошибка); одна или ноль — расстояние до точки.
    """
    if not feet:
        return -9.0
    if len(feet) == 1:
        return -math.hypot(cx - feet[0][0], cy - feet[0][1])
    if len(feet) == 2:
        return -_dist_seg(cx, cy, feet[0], feet[1])
    gx = sum(f[0] for f in feet) / len(feet)
    gy = sum(f[1] for f in feet) / len(feet)
    poly = sorted(feet, key=lambda f: math.atan2(f[1] - gy, f[0] - gx))
    inside = True
    best = 9.0
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        cross = (b[0] - a[0]) * (cy - a[1]) - (b[1] - a[1]) * (cx - a[0])
        if cross < 0:
            inside = False
        best = min(best, _dist_seg(cx, cy, a, b))
    return best if inside else -best


def _dist_seg(px, py, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    wx, wy = px - a[0], py - a[1]
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 <= 1e-12 else max(0.0, min(1.0, (wx * vx + wy * vy) / L2))
    return math.hypot(px - (a[0] + t * vx), py - (a[1] + t * vy))


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a
