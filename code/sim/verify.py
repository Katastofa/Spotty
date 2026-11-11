"""
ВОРОТА МОДЕЛИ: URDF описывает ТУ ЖЕ ногу, что и прошивка.

Проверка устроена так, что подогнать её нельзя. Прошивка выдаёт углы для
точки стопы (x, y, z) своей настоящей legIK; симулятор ставит эти углы в
модель и спрашивает PyBullet, где оказалась стопа. Если URDF собран с другим
знаком, другим порядком звеньев или другим выносом хипа — невязка вылезет
сразу и будет большой, не «почти ноль».

    python verify.py
"""
import os
import sys

import pybullet as p

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spotty_sim import firmware, model as M


def main():
    geom = firmware.geometry()
    m = M.Model().with_geometry(geom["geom_mm"])
    urdf = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spotty_gen.urdf")
    with open(urdf, "w", encoding="utf-8") as f:
        f.write(M.build_urdf(m))          # без пределов: проверяем всю зону
    print("URDF собран: %s" % urdf)

    p.connect(p.DIRECT)
    rid = p.loadURDF(urdf, basePosition=[0, 0, 0], useFixedBase=True)
    jid, lid = {}, {}
    for i in range(p.getNumJoints(rid)):
        info = p.getJointInfo(rid, i)
        jid[info[1].decode()] = i
        lid[info[12].decode()] = i

    T = m.tibia_mm * M.MM
    lat, lon = m.base_lat_mm * M.MM, m.base_long_mm * M.MM

    rows = firmware.ik_grid()
    print("точек из прошивки: %d" % len(rows))
    worst, worst_leg = 0.0, None
    for i, leg in enumerate(M.LEGS):
        s, fw = M.LEG_LEFT[i], M.LEG_FWD[i]
        hip_origin = (fw * lon / 2, s * lat / 2, 0.0)
        for x, y, z, h, t, k in rows:
            q = M.to_urdf_angles(i, h, t, k)
            for name, val in zip(("j_hip_", "j_thigh_", "j_knee_"), q):
                p.resetJointState(rid, jid[name + leg], val)
            st = p.getLinkState(rid, lid["shank_" + leg], computeForwardKinematics=True)
            pos, orn = st[4], st[5]
            rot = p.getMatrixFromQuaternion(orn)
            foot = [pos[j] + rot[3 * j + 2] * (-T) for j in range(3)]
            # обратно в систему прошивки: x вперёд, y наружу, z вниз, мм
            fx = (foot[0] - hip_origin[0]) / M.MM
            fy = s * (foot[1] - hip_origin[1]) / M.MM
            fz = -(foot[2] - hip_origin[2]) / M.MM
            err = max(abs(fx - x), abs(fy - y), abs(fz - z))
            if err > worst:
                worst, worst_leg = err, (leg, x, y, z, fx, fy, fz)
    p.disconnect()

    print("худшая невязка стопы: %.6f мм" % worst)
    if worst_leg:
        leg, x, y, z, fx, fy, fz = worst_leg
        print("  где: %s  заказано (%.1f, %.1f, %.1f)  получилось (%.3f, %.3f, %.3f)"
              % (leg, x, y, z, fx, fy, fz))
    ok = worst < 0.01
    print("[%s] URDF и legIK прошивки — одна и та же нога" % (" OK " if ok else "ПРОВАЛ"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
