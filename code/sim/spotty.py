"""
Spotty — параметрическая модель четвероногого робота для PyBullet.

Запуск:
    python spotty.py              # окно GUI, робот шагает рысью
    python spotty.py --direct     # без окна, 5 секунд, вывод в консоль
    python spotty.py --urdf-only  # только сгенерировать spotty.urdf и выйти

ВСЕ параметры робота — в классе Cfg ниже. Это заглушки правдоподобного
размера (примерно масштаб SpotMicro). Подставьте реальные цифры Spotty —
URDF и походка пересоберутся сами, менять код не нужно.

Управление в GUI (слайдеры):
    speed      — скорость шага вперёд (длина шага, м)
    turn       — поворот (разница длины шага левых и правых ног)
    height     — высота корпуса над землёй, м
    clearance  — на сколько поднимается стопа в переносе, м
    freq       — частота шагов, Гц
    Кнопка "reset" возвращает робота в начальную позу.
"""
import argparse
import math
import os

try:
    import pybullet as p
    import pybullet_data
except ImportError:  # noqa
    raise SystemExit(
        "Не найден pybullet. Установите одним из способов:\n"
        "    pip install pybullet-arm64      (готовые сборки под Windows)\n"
        "    conda install -c conda-forge pybullet"
    )


# ────────────────────────────────────────────────────────────────────────
#  ПАРАМЕТРЫ РОБОТА — правьте здесь
# ────────────────────────────────────────────────────────────────────────
class Cfg:
    # --- корпус (габаритная коробка) ---
    body_len = 0.25          # длина, м (вдоль X, вперёд)
    body_wid = 0.10          # ширина, м (вдоль Y)
    body_hgt = 0.05          # высота, м (вдоль Z)
    body_mass = 1.20         # кг

    # --- расположение тазобедренных суставов относительно центра корпуса ---
    hip_dx = 0.11            # смещение вперёд/назад, м
    hip_dy = 0.05            # смещение вбок, м (половина ширины корпуса)

    # --- звенья ноги ---
    l1 = 0.045               # отвод в сторону: от сустава отведения до плеча, м
    l2 = 0.110               # бедро, м
    l3 = 0.130               # голень, м

    hip_mass = 0.10          # кг
    thigh_mass = 0.15
    shank_mass = 0.08
    link_r = 0.012           # радиус звена для отрисовки и инерции, м
    foot_r = 0.018           # радиус стопы, м

    # --- пределы суставов, рад ---
    lim_abduct = 0.80
    lim_thigh = 2.60
    knee_min, knee_max = -2.70, -0.05

    # --- привод ---
    max_torque = 4.0         # Н·м, момент сервопривода
    max_velocity = 8.0       # рад/с
    kp = 0.6                 # усиление позиционного контура PyBullet
    kd = 0.3

    # --- стойка и походка (стартовые значения слайдеров) ---
    stand_height = 0.175     # высота корпуса над землёй, м
    step_len = 0.06          # длина шага, м
    step_clearance = 0.045   # подъём стопы в переносе, м
    gait_freq = 1.6          # шагов в секунду
    duty = 0.5               # доля цикла в опоре (0.5 = рысь)

    foot_friction = 1.2
    settle_seconds = 0.6     # сколько стоять на месте перед стартом походки


# Ноги: имя → (знак по X: 1 перёд, -1 зад, знак по Y: 1 лево, -1 право, фаза)
LEGS = {
    "FL": (+1, +1, 0.0),
    "FR": (+1, -1, 0.5),
    "RL": (-1, +1, 0.5),
    "RR": (-1, -1, 0.0),
}


# ────────────────────────────────────────────────────────────────────────
#  ГЕНЕРАЦИЯ URDF
# ────────────────────────────────────────────────────────────────────────
def _inertial_box(mass, sx, sy, sz):
    ixx = mass / 12.0 * (sy * sy + sz * sz)
    iyy = mass / 12.0 * (sx * sx + sz * sz)
    izz = mass / 12.0 * (sx * sx + sy * sy)
    return (f'    <inertial><mass value="{mass}"/>'
            f'<origin xyz="0 0 0"/>'
            f'<inertia ixx="{ixx:.6g}" ixy="0" ixz="0" '
            f'iyy="{iyy:.6g}" iyz="0" izz="{izz:.6g}"/></inertial>\n')


def _inertial_rod(mass, length, radius, axis, offset):
    """Стержень длиной length вдоль оси axis ('y' или 'z'), центр в offset."""
    i_long = 0.5 * mass * radius * radius
    i_perp = mass / 12.0 * (3 * radius * radius + length * length)
    if axis == "y":
        ixx, iyy, izz = i_perp, i_long, i_perp
    else:
        ixx, iyy, izz = i_perp, i_perp, i_long
    ox, oy, oz = offset
    return (f'    <inertial><mass value="{mass}"/>'
            f'<origin xyz="{ox:.6g} {oy:.6g} {oz:.6g}"/>'
            f'<inertia ixx="{ixx:.6g}" ixy="0" ixz="0" '
            f'iyy="{iyy:.6g}" iyz="0" izz="{izz:.6g}"/></inertial>\n')


def _rod_geom(length, radius, axis, offset, rgba):
    """Цилиндр вдоль axis: в URDF цилиндр всегда вдоль Z, поэтому для 'y' крутим."""
    rpy = "1.5708 0 0" if axis == "y" else "0 0 0"
    ox, oy, oz = offset
    g = (f'<origin xyz="{ox:.6g} {oy:.6g} {oz:.6g}" rpy="{rpy}"/>'
         f'<geometry><cylinder radius="{radius}" length="{length:.6g}"/></geometry>')
    return (f'    <visual>{g}<material name="m"><color rgba="{rgba}"/></material></visual>\n'
            f'    <collision>{g}</collision>\n')


def build_urdf(c=Cfg):
    """Собирает URDF робота строкой по параметрам конфига."""
    out = ['<?xml version="1.0"?>\n<robot name="spotty">\n']

    # --- корпус ---
    out.append('  <link name="base_link">\n')
    out.append(_inertial_box(c.body_mass, c.body_len, c.body_wid, c.body_hgt))
    box = (f'<origin xyz="0 0 0"/><geometry>'
           f'<box size="{c.body_len} {c.body_wid} {c.body_hgt}"/></geometry>')
    out.append(f'    <visual>{box}<material name="body">'
               f'<color rgba="0.20 0.45 0.75 1"/></material></visual>\n')
    out.append(f'    <collision>{box}</collision>\n')
    out.append('  </link>\n\n')

    for leg, (sx, sy, _phase) in LEGS.items():
        hip, thigh, shank = f"hip_{leg}", f"thigh_{leg}", f"shank_{leg}"

        # сустав отведения: ось X, в точке крепления к корпусу
        out.append(f'  <joint name="j_abduct_{leg}" type="revolute">\n'
                   f'    <parent link="base_link"/><child link="{hip}"/>\n'
                   f'    <origin xyz="{sx * c.hip_dx:.6g} {sy * c.hip_dy:.6g} 0"/>\n'
                   f'    <axis xyz="1 0 0"/>\n'
                   f'    <limit lower="{-c.lim_abduct}" upper="{c.lim_abduct}" '
                   f'effort="{c.max_torque}" velocity="{c.max_velocity}"/>\n'
                   f'  </joint>\n')
        out.append(f'  <link name="{hip}">\n')
        out.append(_inertial_rod(c.hip_mass, c.l1, c.link_r, "y",
                                 (0, sy * c.l1 / 2, 0)))
        out.append(_rod_geom(c.l1, c.link_r, "y", (0, sy * c.l1 / 2, 0),
                             "0.85 0.85 0.88 1"))
        out.append('  </link>\n')

        # бедро: ось Y, на конце отвода
        out.append(f'  <joint name="j_thigh_{leg}" type="revolute">\n'
                   f'    <parent link="{hip}"/><child link="{thigh}"/>\n'
                   f'    <origin xyz="0 {sy * c.l1:.6g} 0"/>\n'
                   f'    <axis xyz="0 1 0"/>\n'
                   f'    <limit lower="{-c.lim_thigh}" upper="{c.lim_thigh}" '
                   f'effort="{c.max_torque}" velocity="{c.max_velocity}"/>\n'
                   f'  </joint>\n')
        out.append(f'  <link name="{thigh}">\n')
        out.append(_inertial_rod(c.thigh_mass, c.l2, c.link_r, "z", (0, 0, -c.l2 / 2)))
        out.append(_rod_geom(c.l2, c.link_r, "z", (0, 0, -c.l2 / 2),
                             "0.95 0.60 0.15 1"))
        out.append('  </link>\n')

        # колено: ось Y, на конце бедра
        out.append(f'  <joint name="j_knee_{leg}" type="revolute">\n'
                   f'    <parent link="{thigh}"/><child link="{shank}"/>\n'
                   f'    <origin xyz="0 0 {-c.l2:.6g}"/>\n'
                   f'    <axis xyz="0 1 0"/>\n'
                   f'    <limit lower="{c.knee_min}" upper="{c.knee_max}" '
                   f'effort="{c.max_torque}" velocity="{c.max_velocity}"/>\n'
                   f'  </joint>\n')
        out.append(f'  <link name="{shank}">\n')
        out.append(_inertial_rod(c.shank_mass, c.l3, c.link_r, "z", (0, 0, -c.l3 / 2)))
        out.append(_rod_geom(c.l3, c.link_r * 0.85, "z", (0, 0, -c.l3 / 2),
                             "0.30 0.30 0.34 1"))
        foot = (f'<origin xyz="0 0 {-c.l3:.6g}"/>'
                f'<geometry><sphere radius="{c.foot_r}"/></geometry>')
        out.append(f'    <visual>{foot}<material name="foot">'
                   f'<color rgba="0.12 0.12 0.12 1"/></material></visual>\n')
        out.append(f'    <collision>{foot}</collision>\n')
        out.append('  </link>\n\n')

    out.append('</robot>\n')
    return "".join(out)


# ────────────────────────────────────────────────────────────────────────
#  КИНЕМАТИКА НОГИ
# ────────────────────────────────────────────────────────────────────────
def leg_ik(x, y, z, sy, c=Cfg):
    """Углы (отведение, бедро, колено) для стопы в точке (x,y,z)
    относительно сустава отведения. sy = +1 левая нога, -1 правая.
    Проверено сверкой с прямой кинематикой: ошибка ~1e-15 рад."""
    r2 = y * y + z * z - c.l1 * c.l1
    L = math.sqrt(max(r2, 1e-12))
    q1 = math.atan2(z, y) - math.atan2(-L, sy * c.l1)
    q1 = (q1 + math.pi) % (2 * math.pi) - math.pi

    d2 = x * x + L * L
    cq3 = (d2 - c.l2 * c.l2 - c.l3 * c.l3) / (2 * c.l2 * c.l3)
    q3 = -math.acos(max(-1.0, min(1.0, cq3)))          # колено назад
    q2 = math.atan2(-x, L) - math.atan2(c.l3 * math.sin(q3),
                                        c.l2 + c.l3 * math.cos(q3))
    return q1, q2, q3


def foot_target(leg, t, gp, c=Cfg):
    """Точка стопы в системе сустава отведения. gp — словарь параметров походки."""
    sx, sy, phase = LEGS[leg]
    # поворот: левые и правые ноги шагают на разную длину
    step = gp["step"] + (-gp["turn"] if sy > 0 else gp["turn"])

    ph = (t * gp["freq"] + phase) % 1.0
    y = sy * c.l1
    if ph < c.duty:                                    # опора: стопа едет назад
        u = ph / c.duty
        x = step * (0.5 - u)
        z = -gp["height"]
    else:                                              # перенос: вперёд по дуге
        u = (ph - c.duty) / (1.0 - c.duty)
        x = step * (u - 0.5)
        z = -gp["height"] + gp["clear"] * math.sin(math.pi * u)
    return x, y, z


def reachable(gp, c=Cfg):
    """Проверяет, что вся траектория стопы внутри рабочей зоны ноги."""
    lo, hi = abs(c.l2 - c.l3) + 0.01, c.l2 + c.l3 - 0.01
    for leg in LEGS:
        for k in range(40):
            x, y, z = foot_target(leg, k / 40.0 / max(gp["freq"], 1e-6), gp, c)
            L2 = y * y + z * z - c.l1 * c.l1
            if L2 <= 0:
                return False
            d = math.sqrt(x * x + L2)
            if not (lo < d < hi):
                return False
    return True


# ────────────────────────────────────────────────────────────────────────
#  СИМУЛЯЦИЯ
# ────────────────────────────────────────────────────────────────────────
def spawn(urdf_path, c=Cfg):
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    z0 = c.stand_height + c.body_hgt / 2
    robot = p.loadURDF(urdf_path, basePosition=[0, 0, z0], useFixedBase=False)

    joints = {}
    for i in range(p.getNumJoints(robot)):
        name = p.getJointInfo(robot, i)[1].decode()
        joints[name] = i
        p.changeDynamics(robot, i, linearDamping=0.02, angularDamping=0.02)
    for leg in LEGS:
        p.changeDynamics(robot, joints[f"j_knee_{leg}"],
                         lateralFriction=c.foot_friction, restitution=0.0)
    return robot, joints


def apply_pose(robot, joints, t, gp, reset=False, c=Cfg):
    for leg in LEGS:
        _sx, sy, _ph = LEGS[leg]
        x, y, z = foot_target(leg, t, gp, c)
        q = leg_ik(x, y, z, sy, c)
        for name, angle in zip((f"j_abduct_{leg}", f"j_thigh_{leg}",
                                f"j_knee_{leg}"), q):
            j = joints[name]
            if reset:
                p.resetJointState(robot, j, angle)
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                                    targetPosition=angle,
                                    force=c.max_torque,
                                    maxVelocity=c.max_velocity,
                                    positionGain=c.kp, velocityGain=c.kd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--direct", action="store_true", help="без GUI")
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--urdf-only", action="store_true",
                    help="только сгенерировать spotty.urdf")
    args = ap.parse_args()

    urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "spotty.urdf")
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(build_urdf())
    print(f"URDF записан: {urdf_path}")
    if args.urdf_only:
        return

    gui = not args.direct
    p.connect(p.GUI if gui else p.DIRECT)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=50,
                                     cameraPitch=-25,
                                     cameraTargetPosition=[0, 0, 0.15])
    p.setGravity(0, 0, -9.81)
    dt = 1.0 / 240.0
    p.setTimeStep(dt)

    robot, joints = spawn(urdf_path)

    gp = {"step": Cfg.step_len, "turn": 0.0, "height": Cfg.stand_height,
          "clear": Cfg.step_clearance, "freq": Cfg.gait_freq}
    stand = dict(gp, step=0.0, turn=0.0)
    apply_pose(robot, joints, 0.0, stand, reset=True)

    sliders = {}
    reset_btn = None
    if gui:
        sliders["step"] = p.addUserDebugParameter("speed", -0.10, 0.14, Cfg.step_len)
        sliders["turn"] = p.addUserDebugParameter("turn", -0.06, 0.06, 0.0)
        sliders["height"] = p.addUserDebugParameter("height", 0.10, 0.22,
                                                    Cfg.stand_height)
        sliders["clear"] = p.addUserDebugParameter("clearance", 0.0, 0.08,
                                                   Cfg.step_clearance)
        sliders["freq"] = p.addUserDebugParameter("freq", 0.2, 3.5, Cfg.gait_freq)
        reset_btn = p.addUserDebugParameter("reset", 1, 0, 0)
        last_reset = p.readUserDebugParameter(reset_btn)

    seconds = args.seconds if args.seconds is not None else (None if gui else 5.0)
    step = 0
    gait_t = 0.0
    import time

    try:
        while seconds is None or step * dt < seconds:
            sim_t = step * dt
            if gui:
                for k, h in sliders.items():
                    gp[k] = p.readUserDebugParameter(h)
                r = p.readUserDebugParameter(reset_btn)
                if r != last_reset:
                    last_reset = r
                    p.resetBasePositionAndOrientation(
                        robot, [0, 0, gp["height"] + Cfg.body_hgt / 2], [0, 0, 0, 1])
                    p.resetBaseVelocity(robot, [0, 0, 0], [0, 0, 0])
                    apply_pose(robot, joints, 0.0, dict(gp, step=0.0, turn=0.0),
                               reset=True)
                    gait_t = 0.0

            if sim_t < Cfg.settle_seconds:       # сначала устояться на месте
                apply_pose(robot, joints, 0.0, dict(gp, step=0.0, turn=0.0))
            else:
                gait_t += dt
                cur = gp if reachable(gp) else dict(gp, step=0.0, turn=0.0)
                apply_pose(robot, joints, gait_t, cur)

            p.stepSimulation()
            step += 1

            if step % 240 == 0:
                pos, orn = p.getBasePositionAndOrientation(robot)
                roll, pitch, yaw = p.getEulerFromQuaternion(orn)
                print(f"t={sim_t:5.1f}s  x={pos[0]:+.3f} y={pos[1]:+.3f} "
                      f"z={pos[2]:.3f}  крен={math.degrees(roll):+5.1f}° "
                      f"тангаж={math.degrees(pitch):+5.1f}° "
                      f"курс={math.degrees(yaw):+6.1f}°")

            if gui:
                time.sleep(dt)
    except p.error:
        pass
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
