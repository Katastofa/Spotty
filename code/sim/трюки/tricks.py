"""
ТРЮКИ SPOTTY — ключевые кадры. Источник правды для code/gait_test/spotty_tricks.h.

    python tricks_gen.py        пересобрать spotty_tricks.h из этого файла
    python проверка.py          прогнать все трюки из ПРОШИВКИ (trace) в PyBullet

Кадр: (мс, [(вперёд, влево, вверх) × FR FL BR BL]) — смещения стоп в осях
корпуса от «канона» (стопа под осью бедра на высоте позы), мм.
Идеи движений: Petoi Bittle (hi, sit, str, pu, ck), Unitree Go2 (Hello, Sit,
Stretch, Dance, WiggleHips), SpotMicro (sit, give paw). Числа — свои, под эту
раму; подобраны в PyBullet (оба движка, 4 разных успокоения, 0 падений из 80)
ПРИ ХИПЕ «+ = НАРУЖУ». С обратным знаком лапа/танец/вилять/оглядеться падают.
"""
import math
Z = [(0,0,0)]*4
LF, LL = [1,1,-1,-1], [-1,1,-1,1]
HX, HY, D0 = 93.0, 39.0, 188.0
def body(tx=0, ty=0, tz=0, roll=0, pitch=0, yaw=0, free=None, pivot=(0.0, 0.0, -D0)):
    """Смещения стоп (вперёд, влево, вверх; оси корпуса) для ЖЁСТКОГО движения
    корпуса при стоящих стопах. roll + = правый борт вниз, pitch + = нос вверх,
    yaw + = нос влево (соглашения прошивки/пульта)."""
    r, p, y = map(math.radians, (roll, pitch, yaw))
    # R = Rz(y) Ry(-p) Rx(r):  нос вверх = поворот вокруг y на -p
    def Rx(v,a): c,s=math.cos(a),math.sin(a); return [v[0], v[1]*c-v[2]*s, v[1]*s+v[2]*c]
    def Ry(v,a): c,s=math.cos(a),math.sin(a); return [v[0]*c+v[2]*s, v[1], -v[0]*s+v[2]*c]
    def Rz(v,a): c,s=math.cos(a),math.sin(a); return [v[0]*c-v[1]*s, v[0]*s+v[1]*c, v[2]]
    # roll + = правый борт вниз: вокруг x на -r? правый борт (y<0) вниз -> точка (0,-1,0) идёт в -z:
    # Rx(+a): (0,-1,0) -> (0,-c,-s): вниз при a>0. Значит Rx(+r).
    # поворот вокруг точки pivot (по умолчанию — на уровне стоп под центром):
    # t_eff = t + c - R c
    c = list(pivot)
    Rc = Rz(Ry(Rx(c, r), -p), y)
    tx, ty, tz = tx + c[0]-Rc[0], ty + c[1]-Rc[1], tz + c[2]-Rc[2]
    k = []
    for leg in range(4):
        pw = [LF[leg]*HX, LL[leg]*HY, -D0]                 # стопа в мире (при нулевой позе корпус в 0)
        q = [pw[0]-tx, pw[1]-ty, pw[2]-tz]                 # в осях корпуса: R^T (p - t)
        q = Rz(q, -y); q = Ry(q, p); q = Rx(q, -r)
        k.append((round(q[0]-pw[0],1), round(q[1]-pw[1],1), round(q[2]-pw[2],1)))
    if free:
        for leg, v in free.items(): k[leg] = v
    return k
def sitk(s=0, fr=None, rz=100):
    """Сед: задние стопы вверх на rz (зад опускается на круп), передние на месте.
    s — сдвиг корпуса влево (мм, хипами), fr — поднятая правая передняя лапа:
    (вперёд, вверх) от своей стоящей точки. Подобрано в PyBullet: 12 из 12
    прогонов без падения (оба движка, три разных успокоения)."""
    k = [(0,-s,0),(0,-s,0),(0,-s,rz),(0,-s,rz)]
    if fr: k[0] = (fr[0], -s, fr[1])
    return k
S = 20
SIT = [(1200, sitk(rz=60)), (1000, sitk())]          # сесть в два приёма: круп опускается мягко
UNSIT = [(1400, sitk(rz=60)), (1800, Z)]
TRICKS = {
 'hiptest':('проверка знака хипа', [(1000, body(ty=15)), (1500, body(ty=15)), (1000, Z)], False),
 'sit':    ('сесть', SIT + [(400, sitk())], True),
 'paw':    ('дать лапу', SIT + [(800, sitk(S)), (900, sitk(S,(60,90))), (1500, sitk(S,(60,90))),
                          (900, sitk(S)), (800, sitk())] + UNSIT, False),
 'wave':   ('помахать', SIT + [(800, sitk(S)), (900, sitk(S,(60,100)))] +
                        [(350, sitk(S,(70,70))), (350, sitk(S,(60,100)))]*3 +
                        [(900, sitk(S)), (800, sitk())] + UNSIT, False),
 'bow':    ('поклон', [(1200, body(pitch=-14, pivot=(-HX,0,-D0))), (1500, body(pitch=-14, pivot=(-HX,0,-D0))), (1200, Z)], False),
 'stretch':('потянуться', [(1200, body(pitch=-16, pivot=(-HX,0,-D0))), (1200, body(pitch=-16, pivot=(-HX,0,-D0))),
                           (1400, body(pitch=12, pivot=(HX,0,-D0))), (1200, body(pitch=12, pivot=(HX,0,-D0))), (1200, Z)], False),
 'pushup': ('отжимания', [(800, body(pitch=-3))] + [(600, body(pitch=-12, pivot=(-HX,0,-D0))), (600, body(pitch=-3))]*3 + [(800, Z)], False),
 'dance':  ('танец', [(380, body(tx=18*math.cos(a*math.pi/4), ty=18*math.sin(a*math.pi/4), tz=8*math.sin(a*math.pi/2),
                                 roll=5*math.sin(a*math.pi/4), pitch=-4*math.cos(a*math.pi/4))) for a in range(17)] + [(600, Z)], False),
 'wiggle': ('вилять', [(320, body(ty=8*s, yaw=-7*s, roll=3*s)) for s in (1,-1)*4] + [(500, Z)], False),
 'look':   ('оглядеться', [(1000, body(yaw=8, pitch=5)), (900, body(yaw=8, pitch=5)), (1400, body(yaw=-8, pitch=5)),
                           (900, body(yaw=-8, pitch=5)), (1000, Z)], False),
}
