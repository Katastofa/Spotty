"""
Модель Spotty для PyBullet: числа + генератор URDF.

ПОЛИТИКА ЧИСЕЛ (SIMULATION.md R3). У каждой величины есть статус:

    ИЗМЕРЕНО          снято на железе
    КАНДИДАТ ПО STL   подгонка по файлам печати, штангенциркулем не проверено
    ПРИНЯТО           взято из головы ради того, чтобы модель считалась

Геометрия звеньев приходит ИЗ ПРОШИВКИ (firmware.geometry()), а не пишется
здесь руками — это требование R2. Здесь живут только те числа, которых в
прошивке нет вовсе: массы, продольная база, трение, модель привода.

ПРОДОЛЬНАЯ БАЗА СНЯТА 2026-09-12: 186 мм, рулетка (M4). До этого дня
симулятор считал на ПРИНЯТЫХ 180 — середине коридора 135…232, который
допускали STL. Все выводы, сделанные ДО этой даты (оба протокола обучения),
опираются на 180 и подлежат перепроверке, если зависят от продольной базы:
тангаж, разворот, запас устойчивости вдоль корпуса, амплитуда переноса.

Значение и статус берутся ИЗ ПРОШИВКИ (with_geometry): величину даёт
BASE_LONG_MM, статус — отдельный символ BASE_LONG_IS_MEASURED. По одной
величине судить нельзя: ненулевая база может быть и подставленной ради
свипа, и измеренной.
"""
from dataclasses import dataclass, field, asdict

MM = 0.001

# Порядок ног ровно как в прошивке: FR, FL, BR, BL.
LEGS = ("FR", "FL", "BR", "BL")
LEG_FWD = (+1.0, +1.0, -1.0, -1.0)
LEG_LEFT = (-1.0, +1.0, -1.0, +1.0)
JOINTS = ("hip", "thigh", "knee")


@dataclass
class Model:
    # --- геометрия: приходит из прошивки, здесь только значения по умолчанию
    femur_mm: float = 108.31          # КАНДИДАТ ПО STL (M2)
    tibia_mm: float = 116.51          # КАНДИДАТ ПО STL (M3)
    hip_mm: float = 10.75             # КАНДИДАТ ПО STL (M1)
    foot_r_mm: float = 19.41          # КАНДИДАТ ПО STL (M3)
    base_lat_mm: float = 78.00        # КАНДИДАТ ПО STL

    # --- продольная база: приходит из прошивки вместе со своим статусом ----
    # Значения ниже — только запасной вариант на случай, если прошивка её не
    # отдала: 180 мм это середина коридора 135…232, который допускали STL.
    base_long_mm: float = 180.0
    base_long_measured: bool = False
    base_long_fw_mm: float = 0.0      # что стоит в самой прошивке; отличие от
                                      # base_long_mm означает ПОДМЕНУ ради свипа

    # --- корпус: ПРИНЯТО ---------------------------------------------------
    body_extra_len_mm: float = 60.0   # ПРИНЯТО: свес рамы за оси хипов
    body_extra_wid_mm: float = 34.0   # ПРИНЯТО
    body_hgt_mm: float = 70.0         # ПРИНЯТО
    body_mass_kg: float = 0.98        # ПРИНЯТО: рама + электроника + АКБ
    body_com_z_mm: float = 0.0        # ПРИНЯТО: ЦТ корпуса относительно оси хипа,
                                      # + вверх. 0 = масса ровно на уровне осей.
    body_com_x_mm: float = 0.0        # ПРИНЯТО: ЦТ корпуса вперёд(+)/назад(−)
                                      # от середины между осями хипов.
    body_com_y_mm: float = 0.0        # ПРИНЯТО: ЦТ корпуса влево(+)/вправо(−).
    hip_mass_kg: float = 0.15         # ПРИНЯТО: серво хипа + кронштейн
    thigh_mass_kg: float = 0.18       # ПРИНЯТО: серво бедра + плечо
    shank_mass_kg: float = 0.05       # ПРИНЯТО: голень + стопа
    link_r_mm: float = 9.0            # ПРИНЯТО: радиус звена для отрисовки

    # --- привод: ПРИНЯТО, паспорта на QY3240MG нет (HARDWARE.md) ----------
    stall_torque_nm: float = 3.92     # ПРИНЯТО: 40 кг·см при 7.4 В, из листинга
    no_load_dps: float = 300.0        # ПРИНЯТО: скорость холостого хода
    servo_kp: float = 20.0            # ПРИНЯТО: жёсткость позиционного контура
    servo_kd: float = 0.80            # ПРИНЯТО: демпфирование, около критического
    latency_ms: float = 12.0          # ПРИНЯТО: кадр ШИМ 3.03 мс + отработка

    # --- контакт: ПРИНЯТО, коэффициент пола не измерялся -------------------
    foot_friction: float = 0.9        # ПРИНЯТО: TPU по неизвестному покрытию
    ground_friction: float = 0.9      # ПРИНЯТО
    restitution: float = 0.0          # ПРИНЯТО

    sim_hz: int = 1000                # шаг решателя, R6: фиксированный

    def status_table(self):
        """Что во что верится. Печатается в каждом отчёте (R3)."""
        return [
            ("бедро / голень / вынос хипа / стопа / поперечная база",
             "КАНДИДАТ ПО STL", "из прошивки, KINEMATICS_AUDIT.md §4"),
            ("ПРОДОЛЬНАЯ база %.0f мм" % self.base_long_mm,
             "ИЗМЕРЕНО" if self.base_long_measured else "ПРИНЯТО",
             "M4, рулетка 2026-09-12" if self.base_long_measured
             else "M4 не снят; допустимо 135…232 мм"),
            ("массы звеньев (итого %.2f кг), ЦТ корпуса %+.0f/%+.0f/%+.0f мм "
             "(вперёд/влево/вверх) от середины осей хипов"
             % (self.total_mass_kg, self.body_com_x_mm, self.body_com_y_mm,
                self.body_com_z_mm),
             "ПРИНЯТО", "не взвешивалось"),
            ("момент серво %.2f Н·м, холостая %.0f °/с" % (
                self.stall_torque_nm, self.no_load_dps),
             "ПРИНЯТО", "паспорта QY3240MG нет, HARDWARE.md"),
            ("жёсткость контура kp=%.1f kd=%.2f, латентность %.0f мс" % (
                self.servo_kp, self.servo_kd, self.latency_ms),
             "ПРИНЯТО", "не измерялось, режим 'z' на железе"),
            ("трение стопы %.2f" % self.foot_friction,
             "ПРИНЯТО", "коэффициент пола неизвестен, смотреть свипом"),
        ]

    @property
    def total_mass_kg(self):
        return self.body_mass_kg + 4 * (
            self.hip_mass_kg + self.thigh_mass_kg + self.shank_mass_kg)

    def with_geometry(self, geom_mm):
        """Подставить геометрию, прочитанную из прошивки."""
        out = Model(**asdict(self))
        out.femur_mm = geom_mm["femur"]
        out.tibia_mm = geom_mm["tibia"]
        out.hip_mm = geom_mm["hip"]
        out.foot_r_mm = geom_mm["foot_r"]
        out.base_lat_mm = geom_mm["base_lat"]
        if geom_mm.get("base_long", 0.0) > 1.0:      # прошивка знает — берём её
            out.base_long_mm = geom_mm["base_long"]
            out.base_long_fw_mm = geom_mm["base_long"]
            # Статус — отдельный факт, а не вывод из величины.
            out.base_long_measured = bool(geom_mm.get("base_long_measured", False))
        return out


# ---------------------------------------------------------------------------
#  ГЕНЕРАЦИЯ URDF
# ---------------------------------------------------------------------------
def _inertial_box(mass, sx, sy, sz, origin=(0, 0, 0)):
    ixx = mass / 12.0 * (sy * sy + sz * sz)
    iyy = mass / 12.0 * (sx * sx + sz * sz)
    izz = mass / 12.0 * (sx * sx + sy * sy)
    return ('    <inertial><mass value="%.6g"/><origin xyz="%.6g %.6g %.6g"/>'
            '<inertia ixx="%.6g" ixy="0" ixz="0" iyy="%.6g" iyz="0" izz="%.6g"/>'
            '</inertial>\n' % (mass, origin[0], origin[1], origin[2], ixx, iyy, izz))


def _inertial_rod(mass, length, radius, axis, origin):
    i_long = 0.5 * mass * radius * radius
    i_perp = mass / 12.0 * (3 * radius * radius + length * length)
    if axis == "y":
        ixx, iyy, izz = i_perp, i_long, i_perp
    else:
        ixx, iyy, izz = i_perp, i_perp, i_long
    return ('    <inertial><mass value="%.6g"/><origin xyz="%.6g %.6g %.6g"/>'
            '<inertia ixx="%.6g" ixy="0" ixz="0" iyy="%.6g" iyz="0" izz="%.6g"/>'
            '</inertial>\n' % (mass, origin[0], origin[1], origin[2], ixx, iyy, izz))


def _rod(length, radius, axis, origin, rgba, collide=True):
    rpy = "1.5708 0 0" if axis == "y" else "0 0 0"
    g = ('<origin xyz="%.6g %.6g %.6g" rpy="%s"/><geometry>'
         '<cylinder radius="%.6g" length="%.6g"/></geometry>'
         % (origin[0], origin[1], origin[2], rpy, radius, length))
    s = '    <visual>%s<material name="m"><color rgba="%s"/></material></visual>\n' % (g, rgba)
    if collide:
        s += '    <collision>%s</collision>\n' % g
    return s


def build_urdf(m: Model, limits_deg=None):
    """URDF по модели.

    СИСТЕМА КООРДИНАТ. База: x вперёд, y влево, z вверх. Нулевая поза — все
    ноги прямые вниз, что в терминах прошивки hip=0, thigh=0, knee=0.

    СВЯЗЬ УГЛОВ С ПРОШИВКОЙ (проверяется verify.py, а не принимается на веру):
        q_hip   = -sign_left * hip_deg    (ось x)
        q_thigh = -thigh_deg              (ось y)
        q_knee  = -knee_deg               (ось y)
    """
    F, T, LH = m.femur_mm * MM, m.tibia_mm * MM, m.hip_mm * MM
    R = m.foot_r_mm * MM
    lat, lon = m.base_lat_mm * MM, m.base_long_mm * MM
    bl = lon + m.body_extra_len_mm * MM
    bw = lat + m.body_extra_wid_mm * MM
    bh = m.body_hgt_mm * MM
    lr = m.link_r_mm * MM

    out = ['<?xml version="1.0"?>\n<!-- СГЕНЕРИРОВАНО spotty_sim/model.py. РУКАМИ НЕ ПРАВИТЬ. -->\n',
           '<robot name="spotty">\n', '  <link name="base_link">\n']
    out.append(_inertial_box(m.body_mass_kg, bl, bw, bh,
                             origin=(m.body_com_x_mm * MM, m.body_com_y_mm * MM,
                                     m.body_com_z_mm * MM)))
    box = '<origin xyz="0 0 0"/><geometry><box size="%.6g %.6g %.6g"/></geometry>' % (bl, bw, bh)
    out.append('    <visual>%s<material name="body"><color rgba="0.20 0.45 0.75 1"/></material></visual>\n' % box)
    out.append('    <collision>%s</collision>\n  </link>\n\n' % box)

    huge = 3.2
    for i, leg in enumerate(LEGS):
        s = LEG_LEFT[i]          # +1 левая, -1 правая
        fw = LEG_FWD[i]
        if limits_deg:
            (h_lo, h_hi), (t_lo, t_hi), (k_lo, k_hi) = limits_deg[i]
            # знак связи «угол прошивки -> угол URDF» переворачивает границы
            jh = (-s * h_hi, -s * h_lo) if s > 0 else (-s * h_lo, -s * h_hi)
            jt, jk = (-t_hi, -t_lo), (-k_hi, -k_lo)
            jh = tuple(sorted(x * 3.14159265 / 180.0 for x in jh))
            jt = tuple(sorted(x * 3.14159265 / 180.0 for x in jt))
            jk = tuple(sorted(x * 3.14159265 / 180.0 for x in jk))
        else:
            jh = jt = jk = (-huge, huge)

        hip, thigh, shank = "hip_" + leg, "thigh_" + leg, "shank_" + leg
        eff, vel = m.stall_torque_nm, m.no_load_dps * 3.14159265 / 180.0

        out.append('  <joint name="j_hip_%s" type="revolute">\n'
                   '    <parent link="base_link"/><child link="%s"/>\n'
                   '    <origin xyz="%.6g %.6g 0"/><axis xyz="1 0 0"/>\n'
                   '    <limit lower="%.6g" upper="%.6g" effort="%.6g" velocity="%.6g"/>\n'
                   '  </joint>\n' % (leg, hip, fw * lon / 2, s * lat / 2, jh[0], jh[1], eff, vel))
        out.append('  <link name="%s">\n' % hip)
        out.append(_inertial_rod(m.hip_mass_kg, LH, lr, "y", (0, s * LH / 2, 0)))
        out.append(_rod(LH, lr, "y", (0, s * LH / 2, 0), "0.85 0.85 0.88 1", collide=False))
        out.append('  </link>\n')

        out.append('  <joint name="j_thigh_%s" type="revolute">\n'
                   '    <parent link="%s"/><child link="%s"/>\n'
                   '    <origin xyz="0 %.6g 0"/><axis xyz="0 1 0"/>\n'
                   '    <limit lower="%.6g" upper="%.6g" effort="%.6g" velocity="%.6g"/>\n'
                   '  </joint>\n' % (leg, hip, thigh, s * LH, jt[0], jt[1], eff, vel))
        out.append('  <link name="%s">\n' % thigh)
        out.append(_inertial_rod(m.thigh_mass_kg, F, lr, "z", (0, 0, -F / 2)))
        out.append(_rod(F, lr, "z", (0, 0, -F / 2), "0.95 0.60 0.15 1", collide=False))
        out.append('  </link>\n')

        out.append('  <joint name="j_knee_%s" type="revolute">\n'
                   '    <parent link="%s"/><child link="%s"/>\n'
                   '    <origin xyz="0 0 %.6g"/><axis xyz="0 1 0"/>\n'
                   '    <limit lower="%.6g" upper="%.6g" effort="%.6g" velocity="%.6g"/>\n'
                   '  </joint>\n' % (leg, thigh, shank, -F, jk[0], jk[1], eff, vel))
        out.append('  <link name="%s">\n' % shank)
        out.append(_inertial_rod(m.shank_mass_kg, T, lr * 0.8, "z", (0, 0, -T / 2)))
        out.append(_rod(T, lr * 0.8, "z", (0, 0, -T / 2), "0.30 0.30 0.34 1", collide=False))
        foot = '<origin xyz="0 0 %.6g"/><geometry><sphere radius="%.6g"/></geometry>' % (-T, R)
        out.append('    <visual>%s<material name="foot"><color rgba="0.12 0.12 0.12 1"/></material></visual>\n' % foot)
        out.append('    <collision>%s</collision>\n' % foot)
        out.append('  </link>\n\n')

    out.append('</robot>\n')
    return "".join(out)


def to_urdf_angles(leg_index, hip_deg, thigh_deg, knee_deg):
    """Углы прошивки (градусы) -> углы суставов URDF (радианы)."""
    d = 3.141592653589793 / 180.0
    s = LEG_LEFT[leg_index]
    return (-s * hip_deg * d, -thigh_deg * d, -knee_deg * d)
