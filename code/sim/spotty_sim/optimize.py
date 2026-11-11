"""
Обучение — подбор параметров походки.

Алгоритм: метод перекрёстной энтропии (CEM), он же «эволюция по элите».
Выбран намеренно простой, потому что его видно насквозь:

    1. взять облако параметров: центр mu, разброс sigma;
    2. насыпать из него lambda вариантов, каждый прогнать в симуляторе;
    3. взять лучшую четверть (элиту) и посчитать по ней новые mu и sigma;
    4. повторить.

Никакой нейросети, никакого обучения с подкреплением: оптимизируются те же
шесть чисел, которые оператор крутит на роботе командами. Значит и результат
— это набор чисел, которые можно ввести с клавиатуры, а не веса, которые
некуда деть.

ДЕТЕРМИНИЗМ (R6). Один и тот же seed даёт один и тот же лог: физика с
фиксированным шагом, генератор случайных чисел свой, порядок оценок не
зависит от того, сколько процессов считает.
"""
import json
import math
import os
import random
import time
from multiprocessing import Pool

from . import evaluate as E

_W = {}


def _worker_init(base_long, gait, cycles, weights, mode="base", fixed=None,
                 grid=None, gui=False, realtime=False):
    """Каждый процесс поднимает свою физику один раз.

    С grid физик не одна, а по одной на каждый набор ПРИНЯТЫХ чисел. Они
    строятся ЗДЕСЬ, один раз на процесс, а не на каждую оценку: пересборка
    мира в PyBullet стоит дороже самого прогона.
    """
    import os as _os
    import tempfile
    from . import model as _M
    base = tempfile.mkdtemp(prefix="spotty_work_")
    _W["worlds"] = []
    for i, over in enumerate(grid or [{}]):
        d = _os.path.join(base, "w%d" % i)
        _os.makedirs(d, exist_ok=True)
        m, urdf, geom = E.prepare_urdf(d, base_long_mm=base_long)
        for k, v in over.items():
            setattr(m, k, v)
        with open(urdf, "w", encoding="utf-8") as f:
            f.write(_M.build_urdf(m, limits_deg=E._limits(geom)))
        _W["worlds"].append((m, urdf, over))
    _W["gait"], _W["cycles"], _W["weights"] = gait, cycles, weights
    _W["mode"], _W["fixed"] = mode, fixed or {}
    # Окно открывает ТОЛЬКО нулевой мир — номинальный. Шесть окон на шесть
    # миров бесполезны: смотреть можно в одно, а рисование стоит времени.
    _W["gui"], _W["realtime"] = gui, realtime


def _caption(m, urdf, text):
    """Подпись в окне: что именно сейчас считается.

    Без неё окно показывает безымянного робота, и непонятно, это удачный
    кандидат или очередной падающий. Обёрнуто в try: картинка — приятное
    дополнение, но ронять из-за неё поиск нельзя.
    """
    try:
        import pybullet as p
        sim = E._sim(m, urdf, gui=True)
        old = _W.get("label")
        if old is not None:
            p.removeUserDebugItem(old, physicsClientId=sim.cid)
        _W["label"] = p.addUserDebugText(text, [0, 0, 0.42],
                                         textColorRGB=[0, 0, 0], textSize=1.2,
                                         physicsClientId=sim.cid)
    except Exception:
        pass


def _worker_eval(x):
    """Оценка кандидата. С одним миром — как было; с сеткой — по всем.

    ЧЕМ СВОДИТСЯ СЕТКА. Не средним: средний балл охотно покупает дальний путь
    при одной удачной массе ценой падения при другой, а массы у нас не
    измерены, и какая из них настоящая — неизвестно. Берём

        среднее МИНУС половина разброса,

    то есть набор, одинаково приличный во всех мирах, выигрывает у набора,
    блестящего в одном и падающего в другом. Плюс в метриках едет честное
    'сколько миров устояло' — по нему и читается результат.
    """
    params = E.decode(_W["mode"], x, _W["fixed"])
    scores, mets = [], []
    for wi, (m, urdf, over) in enumerate(_W["worlds"]):
        show = bool(_W.get("gui")) and wi == 0
        if show:
            keys = sorted(E.space_of(_W["mode"]))
            _caption(m, urdf, "%s | %s" % (
                _W.get("where", ""),
                "  ".join("%s %.0f" % (k, v) for k, v in zip(keys, x))))
        try:
            score, met, summ, full = E.evaluate(params, m, urdf,
                                                cycles=_W["cycles"],
                                                gait=_W["gait"],
                                                weights=_W["weights"],
                                                gui=show,
                                                realtime=show and _W.get("realtime"))
        except Exception as exc:                   # прогон не должен ронять поиск
            return -99.0, {"error": str(exc)[:200]}, {}
        scores.append(score)
        mets.append(met)
    n = len(scores)
    mean = sum(scores) / n
    if n == 1:
        return scores[0], mets[0], summ
    var = sum((s - mean) ** 2 for s in scores) / n
    agg = mean - 0.5 * math.sqrt(var)
    ok = sum(1 for mt in mets if not mt["fell"])
    xs = sorted(mt["x_m"] for mt in mets)
    out = {
        "fell": ok < n,                 # «упал» = упал ХОТЯ БЫ В ОДНОМ мире
        "ok": ok, "n": n,
        "x_m": xs[n // 2],
        "x_m_worst": xs[0],
        "score_mean": mean,
        "score_worst": min(scores),
        "margin_mean_mm": sum(mt["margin_mean_mm"] for mt in mets) / n,
        "track_deg": sum(mt["track_deg"] for mt in mets) / n,
        "roll_rms_deg": sum(mt["roll_rms_deg"] for mt in mets) / n,
        "slip_mm": sum(mt["slip_mm"] for mt in mets) / n,
        "energy_j": sum(mt["energy_j"] for mt in mets) / n,
    }
    return agg, out, summ


def _hms(sec):
    sec = int(max(0, sec))
    if sec < 60:
        return "%d с" % sec
    if sec < 3600:
        return "%d:%02d" % (sec // 60, sec % 60)
    return "%d:%02d:%02d" % (sec // 3600, (sec % 3600) // 60, sec % 60)


def _clip(x, space):
    keys = sorted(space)
    return [min(max(v, space[k][0]), space[k][1]) for k, v in zip(keys, x)]


def cem(gens=20, lam=12, elite=0.25, seed=1, gait=0, cycles=4.0, base_long=None,
        weights=None, workers=2, start=None, log_path=None, verbose=True,
        mode="base", fixed=None, grid=None, gui=False, realtime=False):
    space = E.space_of(mode)
    keys = sorted(space)
    rng = random.Random(seed)
    mu = [float(start[k]) if start and k in start else
          0.5 * (space[k][0] + space[k][1]) for k in keys]
    sigma = [0.35 * (space[k][1] - space[k][0]) for k in keys]
    n_elite = max(3, int(round(lam * elite)))

    history, best = [], None
    t0 = time.time()

    # С ОКНОМ — БЕЗ ПУЛА, И ЭТО НЕ ЛЕНЬ.
    #
    # Пул — это отдельные процессы; окно, открытое в дочернем процессе, живёт
    # своей жизнью: не закрывается по Ctrl+C, а на Windows (там spawn, а не
    # fork) ещё и открывается заново на каждый процесс. Поэтому при gui поиск
    # идёт последовательно в главном процессе: одно окно, закрывается вместе
    # с программой, и видно РОВНО ТОТ кандидат, который считается сейчас.
    # Платим скоростью: без пула нет параллельности.
    if gui:
        _worker_init(base_long, gait, cycles, weights, mode, fixed, grid,
                     gui=True, realtime=realtime)
        pool = None
        run_batch = lambda cl: [_worker_eval(x) for x in cl]
    else:
        pool = Pool(workers, initializer=_worker_init,
                    initargs=(base_long, gait, cycles, weights, mode, fixed,
                              grid, False, False))
        run_batch = lambda cl: pool.map(_worker_eval, cl)
    try:
        for g in range(gens):
            cloud = []
            for i in range(lam):
                if g == 0 and i == 0 and start:
                    cloud.append(_clip(mu, space))  # первым — то, с чего начали
                else:
                    cloud.append(_clip([rng.gauss(mu[j], sigma[j])
                                        for j in range(len(keys))], space))
            if gui:
                # Подпись обновляется перед КАЖДЫМ кандидатом, поэтому
                # считаем их здесь, а не внутри оценки.
                res = []
                for i, x in enumerate(cloud):
                    # Латиницей: шрифт отладочного слоя PyBullet кириллицу
                    # не рисует, русская подпись стала бы пустым местом.
                    _W["where"] = "gen %d/%d  cand %d/%d" % (g + 1, gens,
                                                             i + 1, lam)
                    res.append(_worker_eval(x))
            else:
                res = run_batch(cloud)
            scored = sorted(zip([r[0] for r in res], cloud, res),
                            key=lambda t: -t[0])
            top = scored[:n_elite]
            mu = [sum(t[1][j] for t in top) / n_elite for j in range(len(keys))]
            sigma = [max(0.02 * (space[keys[j]][1] - space[keys[j]][0]),
                         math.sqrt(sum((t[1][j] - mu[j]) ** 2 for t in top) / n_elite))
                     for j in range(len(keys))]
            gbest = scored[0]
            if best is None or gbest[0] > best[0]:
                best = (gbest[0], dict(zip(keys, gbest[1])), gbest[2][1], gbest[2][2])
            row = {
                "gen": g,
                "best": gbest[0],
                "mean_elite": sum(t[0] for t in top) / n_elite,
                "median": scored[len(scored) // 2][0],
                "fell": sum(1 for t in scored if t[2][1].get("fell")),
                "best_ok": gbest[2][1].get("ok"),
                "best_n": gbest[2][1].get("n"),
                "mu": dict(zip(keys, mu)),
                "sigma": dict(zip(keys, sigma)),
                "best_params": dict(zip(keys, gbest[1])),
                "best_metrics": gbest[2][1],
            }
            history.append(row)
            if verbose:
                # ПРОГРЕСС ЧИТАЕМЫЙ, а не «что-то считается».
                #   миров   в скольких мирах устоял ЛУЧШИЙ кандидат поколения;
                #           это и есть цель, а не балл;
                #   осталось  оценка по фактическому времени поколения.
                el = time.time() - t0
                left = el / (g + 1) * (gens - g - 1)
                ok = row.get("best_ok")
                n = row.get("best_n")
                bar = "#" * int(round(20.0 * (g + 1) / gens))
                print("пок %2d/%d [%-20s] лучший %+7.3f | миров %s | элита %+7.3f"
                      " | %s | прошло %s, осталось ~%s"
                      % (g + 1, gens, bar, row["best"],
                         ("%d/%d" % (ok, n)) if n else "-",
                         row["mean_elite"],
                         " ".join("%s=%.0f" % (k, mu[i]) for i, k in enumerate(keys)),
                         _hms(el), _hms(left)), flush=True)
            if log_path:
                with open(log_path, "w", encoding="utf-8") as f:
                    json.dump({"history": history,
                               "best": {"score": best[0], "params": best[1],
                                        "metrics": best[2], "firmware": best[3]},
                               "seed": seed, "gait": gait, "cycles": cycles,
                               "mode": mode, "fixed": fixed,
                               "lambda": lam, "gens": gens}, f,
                              ensure_ascii=False, indent=1)
    finally:
        if pool is not None:
            pool.close()
            pool.join()
    return best, history
