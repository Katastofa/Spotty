"""
Проверка трюков в физике: траектории берутся из НАСТОЯЩЕЙ прошивки
(стенд trace, trick=N — тот же rampModeTick, что на роботе) и проигрываются
в PyBullet (spotty_sim.player, модель серво с моментом и латентностью).

    python проверка.py              хип «+ = наружу» (замер) — так трюки и подобраны
    python проверка.py --inward     обратный знак хипа: что будет, если замер неверен

Падение: крен > 45°, тангаж > 50° или корпус ниже 60 мм. Каждый трюк —
угловой и декартов движок × четыре длительности успокоения.
Массы, момент серво, трение — ПРИНЯТЫЕ (spotty_sim/model.py). Это проверка
на грубые ошибки, а не гарантия: первый раз на железе — на подвесе.
"""
import contextlib, io, json, os, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from spotty_sim import evaluate as E, firmware, player as PL, model as M

B = firmware.build("trace")
m, urdf0, geom = E.prepare_urdf(os.path.join(HERE, '..', 'spotty_sim'))

def urdf_for(hip_out):
    # URDF симулятора: хип + = ВНУТРЬ (как legIK). Замер: + = НАРУЖУ. Для замера
    # переворачиваем знак хипа в командах И его пределы в URDF.
    if not hip_out:
        return urdf0
    L = geom['limits_deg']
    lim = [[[-L[3*i][1], -L[3*i][0]], L[3*i+1], L[3*i+2]] for i in range(4)]
    u = os.path.join(HERE, 'spotty_hip_out.urdf')
    with open(u, 'w', encoding='utf-8') as f:
        f.write(M.build_urdf(m, limits_deg=lim))
    return u

def main():
    hip_out = '--inward' not in sys.argv
    U = urdf_for(hip_out)
    names = [t['name'] for t in json.loads(subprocess.run([B, 'geom=1'], capture_output=True, text=True).stdout)['tricks']]
    csv = os.path.join(HERE, '_trick.csv')
    print('хип для трюков: %s' % ('+ = НАРУЖУ (замер)' if hip_out else '+ = ВНУТРЬ (обратный знак)'))
    for i, nm in enumerate(names):
        res = []
        for eng in (0, 1):
            for st in (300, 500, 700, 900):
                r = subprocess.run([B, 'engine=%d' % eng, 'trick=%d' % i, 'settle=%d' % st, 'out=' + csv],
                                   capture_output=True, text=True)
                d = json.loads(r.stdout)
                ts, rows, settle = PL.read_csv(csv)
                rows = rows + [rows[-1]] * 500
                ts = [k * 0.003 for k in range(len(rows))]
                if hip_out:
                    rows = [[(-v if j % 3 == 0 else v) for j, v in enumerate(rw)] for rw in rows]
                sim = PL.Sim(m, U)
                with contextlib.redirect_stdout(io.StringIO()):
                    o = sim.run(ts, rows, settle, 190 if eng else 186, record=True)
                s = o['series']; sim.close()
                fall = any(abs(a) > 45 for a in s['roll']) or any(abs(a) > 50 for a in s['pitch']) or min(s['z']) < 0.06
                res.append((fall, max(abs(a) for a in s['roll']), max(abs(a) for a in s['pitch']), d['limit_hits']))
        print('%-8s падений %d/8  крен макс %4.1f°  тангаж макс %4.1f°  клампов %d'
              % (nm, sum(x[0] for x in res), max(x[1] for x in res), max(x[2] for x in res), sum(x[3] for x in res)), flush=True)
    os.remove(csv)

if __name__ == '__main__':
    main()
