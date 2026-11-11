# -*- coding: utf-8 -*-
"""
СБОРКА ВНЕШНЕГО ВИДА Spotty из STL — для пульта (пульт_3d.html).

    python сборка_stl.py            -> сборка_stl.json рядом

Пишет, КУДА в звенья URDF (base_link, hip_*, thigh_*, shank_*) кладётся
каждая печатная деталь SpotPupPi / SpotMicro KDY0523 (архив
«SpotPupPi (Spotmini) - 3638679 - part 1 of 2.zip» в корне проекта).
Сами STL сервер пульта читает прямо из архива, распаковывать не нужно.

ЭТО ВНЕШНИЙ ВИД, А НЕ МОДЕЛЬ. Кинематика в пульте по-прежнему URDF из
прошивки. Детали привязаны к её осям так:

  ИЗ STL (KINEMATICS_AUDIT.md §3, подгонка окружностей, невязка ~0):
    оси суставов в каждой детали; ось бедра хипа; колено X=-3.819 Z=116.925
    в arm; колено X=9.5 Z=10 и гнездо стопы X=126.01 Z=10.01 в wrist;
    плечи: оси хипов X=16/94 Z=26; протектор стопы R=19.41.
  ВЫВЕДЕНО ПО ГЕОМЕТРИИ ДЕТАЛЕЙ:
    какая деталь на какую ногу (L/R), какой стороной наружу, куда смотрит
    вал каждого серво; arm_joint: FL, BR — L, FR, BL — R (по диагонали).
  ПРИНЯТО (не измерено, ошибка до ~5 мм):
    положения деталей ВДОЛЬ осей — по стопке «фланец серво -> вал ->
    качалка» с размерами MG996R; крышки корпуса и голова/хвост по высоте.

  РАСХОЖДЕНИЕ С МОДЕЛЬЮ ПРОШИВКИ, видное в сборке:
    L_HIP = 10.75 в STL — это расстояние между осью хипа и осью бедра по
    ВЕРТИКАЛИ (общий перпендикуляр двух скрещивающихся осей). legIK и URDF
    считают его БОКОВЫМ выносом плоскости ноги. По сборке плоскость ноги
    (середина стопы) — примерно в 55 мм наружу от оси хипа, то есть стопы
    стоят на ~188 мм друг от друга, а модель считает 99.5 мм. Сетка поэтому
    стоит шире красных точек стоп URDF. Проверка — линейкой, см. отчёт.

  Чтобы детали сошлись с кинематикой, ось бедра детали совмещена с осью
  бедра URDF, а корпус и arm_joint подняты на 10.75 мм (ZH).
"""
import json, math, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

LH, F, T = 10.75, 108.31, 116.51
LAT, LON = 78.0, 186.0
LEGS = ["FR", "FL", "BR", "BL"]
FWD = {"FR": 1, "FL": 1, "BR": -1, "BL": -1}
LEFT = {"FR": -1, "FL": 1, "BR": -1, "BL": 1}
ZH = 10.75          # сетка корпуса выше оси хипа URDF: ось бедра детали совмещена с осью URDF

def basis(mapping):
    """mapping: {'X':vec,'Y':vec,'Z':vec} -> R (столбцы — куда идут оси детали)."""
    R = np.column_stack([np.array(mapping[a], float) for a in "XYZ"])
    assert abs(np.linalg.det(R) - 1) < 1e-9, ("не собственный поворот", mapping, np.linalg.det(R))
    return R

def rot_y(phi):
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def place(R, part_pt, link_pt):
    t = np.array(link_pt, float) - R @ np.array(part_pt, float)
    return R, t

X, Y, Z = np.eye(3)
parts = []
def add(name, file, link, R, t, color, note="", anchors=None):
    """anchors — точки скелета в координатах ДЕТАЛИ (для вида «чертёж»)."""
    d = {"name": name, "file": file, "link": link,
         "R": np.round(R, 6).tolist(), "t": np.round(t, 3).tolist(),
         "color": color, "note": note}
    if anchors:
        d["anchors"] = {k: [round(float(x), 3) for x in v] for k, v in anchors.items()}
    parts.append(d)

WHITE, DARK, BLACK, GREY = "#e9eaec", "#5b606b", "#474b54", "#9aa0ab"   # тёмное светлее натуры: фон пульта тёмный

for L in LEGS:
    s, fw = LEFT[L], FWD[L]
    # ---- хип: arm_joint. ось бедра детали (X, Y=10.25, Z=20.25) -> ось бедра URDF
    useL = (s * fw == 1)                      # FL, BR — L; FR, BL — R (диагональ)
    if useL:
        R = basis({"X": s * Y, "Y": fw * X, "Z": np.cross(s * Y, fw * X)})
        A = (10.0, 10.25, 20.25)             # точка оси хипа по X, на оси бедра
        f = "L_arm_joint.stl"
    else:
        R = basis({"X": -s * Y, "Y": fw * X, "Z": np.cross(-s * Y, fw * X)})
        A = (-40.0, 10.25, 20.25)
        f = "R_arm_joint.stl"
    assert np.allclose(R @ Z, -Z), (L, R @ Z)
    Ahip = (A[0], A[1], 9.5)                   # на оси хипа детали
    add("arm_joint_" + L, f, "hip_" + L, *place(R, A, (0, 0, 0)), DARK,
        "ось хипа детали на 10.75 выше оси URDF, ось бедра совпадает", {"hip": Ahip})

    # u — расстояние наружу вдоль оси бедра от оси хипа (мм); y в звене бедра = s*u - s*LH
    def y_leg(u): return s * u - s * LH

    # ---- бедро: arm (+ крышка). Y детали -> +y у обеих сторон (L слева, R справа)
    d = np.array([-13.819, 107.425])          # ось бедра -> ось колена в (X,Z) детали
    base = basis({"X": X, "Y": Y, "Z": Z})
    # поворот вокруг y: (dX,dZ) -> (0,-|d|)
    phi = math.atan2(d[0], -d[1])              # (dX,dZ) -> (0,-|d|)
    Rarm = rot_y(phi)
    v = Rarm @ np.array([d[0], 0, d[1]])
    assert abs(v[0]) < 1e-6 and v[2] < 0, v
    if s > 0:
        f, fc, Yu = "L_arm.stl", "L_arm_cover.stl", (lambda Yp: Yp + 46.3)    # u(Y)
        Y0 = 0.0
    else:
        f, fc, Yu = "R_arm.stl", "R_arm_cover.stl", (lambda Yp: -Yp - 3.7)
        Y0 = -50.0
    # Y детали -> +y: для правой ноги наружу — это -Y, т.е. u убывает с Y, y=-u растёт ✓
    pt = (10.0, Y0, 9.5)
    add("arm_" + L, f, "thigh_" + L, *place(Rarm, pt, (0, y_leg(Yu(Y0)), 0)), WHITE,
        "ось бедра: X=10 Z=9.5; колено X=-3.819 Z=116.925 (аудит §3)",
        {"thigh": (10.0, 15.0 if s > 0 else -65.0, 9.5)})
    # крышка: у L_arm_cover и L_arm общая система координат STL (крышка ложится
    # на край Y=29.8 -> 30). R_arm_cover отзеркален со сдвигом -25, R_arm — -50.
    Yc = (lambda Yp: Yp + 46.3) if s > 0 else (lambda Yp: -Yp + 21.3)
    ptc = (10.0, 0.0, 9.5)
    add("arm_cover_" + L, fc, "thigh_" + L, *place(Rarm, ptc, (0, y_leg(Yc(0.0)), 0)), WHITE,
        "крышка бедра, в системе координат arm")

    # ---- голень: wrist. Y детали -> -y у обеих сторон
    dw = np.array([116.51, 0.01])
    flip = basis({"X": X, "Y": -Y, "Z": -Z})   # 180° вокруг x
    # после flip вектор (dX,0,dZ) -> (dX,0,-dZ); повернуть вокруг y в (0,0,-T)
    phw = math.atan2(dw[0], dw[1])            # после flip dZ -> -dZ
    Rw = rot_y(phw) @ flip
    v = Rw @ np.array([dw[0], 0, dw[1]])
    assert abs(v[0]) < 1e-6 and v[2] < 0, v
    assert np.allclose(Rw @ Y, -Y)
    if s > 0:
        f, Yu_w, Y0 = "L_wrist.stl", (lambda Yp: 15.3 - Yp), -30.0
        fY_u = (lambda Yp: 65.3 - Yp)
    else:
        f, Yu_w, Y0 = "R_wrist.stl", (lambda Yp: Yp + 45.3), 0.0
        fY_u = (lambda Yp: Yp + 45.3)
    add("wrist_" + L, f, "shank_" + L, *place(Rw, (9.5, Y0, 10.0), (0, y_leg(Yu_w(Y0)), 0)), BLACK,
        "ось колена X=9.5 Z=10; гнездо стопы X=126.01 Z=10.01 (аудит §3)",
        {"knee": (9.5, -48.0 if s > 0 else 18.0, 10.0)})
    # ---- стопа: 180° вокруг z; протектор к полу
    Rf = basis({"X": -X, "Y": -Y, "Z": Z})
    add("foot_" + L, "foot.stl", "shank_" + L, *place(Rf, (0, 10.0, 0), (0, y_leg(fY_u(10.0)), -T)), GREY,
        "ось гнезда стопы; протектор R=19.41 вниз", {"foot": (0.0, 10.0, 0.0)})

# ---- корпус (base_link), ось хипа сетки на высоте ZH
for fw, tag in ((1, "F"), (-1, "B")):
    Rz = np.diag([fw, fw, 1.0])               # задний конец — поворот на 180° вокруг z
    hx = fw * LON / 2
    R = Rz @ basis({"X": Y, "Y": -X, "Z": Z})
    # F_shoulder: внутренняя грань Y=5 на x = hx + fw*23.25; отверстие Z=26 -> ZH; X=55 -> y=0
    add("F_shoulder_" + tag, "F_shoulder.stl", "base_link",
        *place(R, (55, 5, 26), (hx + fw * 23.25, 0, ZH)), WHITE, "плита качалки хипа")
    add("R_shoulder_" + tag, "R_shoulder.stl", "base_link",
        *place(R, (55, 0, 26), (hx - fw * 22.75, 0, ZH)), WHITE, "плита подшипника F625zz, полки внутрь")
    if fw > 0:
        Rc = basis({"X": X, "Y": Y, "Z": Z})
        add("F_cover", "F_cover.stl", "base_link", *place(Rc, (25, 57, 0), (hx + 28.25, 0, -26 + ZH)), WHITE,
            "ДОПУЩЕНИЕ: голова перед плитой качалки")
    else:
        # хвост: чашка, открытая по +Z детали; скруглённый торец Z=-29.8 — назад
        Rc = basis({"X": Z, "Y": -Y, "Z": X})
        add("R_cover", "R_cover.stl", "base_link", *place(Rc, (0, 57, 40), (hx - 28.25, 0, -26 + ZH)), WHITE,
            "ДОПУЩЕНИЕ: хвост за задней плитой, открытой стороной к корпусу")
Rcov = basis({"X": X, "Y": Z, "Z": -Y})
add("B_cover", "B_cover.stl", "base_link", *place(Rcov, (120, 0, 57), (0, 0, -26 + ZH - 2)), WHITE,
    "ДОПУЩЕНИЕ: низ крышки на 2 мм ниже плит плеч")
add("T_cover", "T_cover.stl", "base_link", *place(Rcov, (120, 124, 57), (0, 0, 70 - 26 + ZH + 2)), WHITE,
    "ДОПУЩЕНИЕ: верх крышки на 2 мм выше плит плеч")

out = {
    "source": "SpotPupPi (Spotmini) - 3638679 - part 1 of 2.zip",
    "zip_dir": "files/",
    "geom_mm": {"hip": LH, "femur": F, "tibia": T, "base_lat": LAT, "base_long": LON},
    "mesh_z_lift_mm": ZH,
    "note": "внешний вид; кинематика — URDF прошивки. См. сборка_stl.py",
    "parts": parts,
}
path = os.path.join(HERE, "сборка_stl.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("%d деталей -> %s" % (len(parts), path))
