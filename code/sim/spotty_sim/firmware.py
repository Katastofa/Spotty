"""
Мост к прошивке. Питоновской копии походки нет и не будет (SIMULATION.md R1):
траектория суставов приходит из code/hostcheck, который собирает настоящий
gait_test.ino обычным g++.

Здесь только вызовы: собрать бинарь один раз, потом дёргать его сотни раз.
"""
import json
import os
import subprocess
import tempfile

# КОДИРОВКА ВЫВОДА СТЕНДА — НЕ МЕЛОЧЬ.
#
# Стенд печатает JSON с русскими строками (например base_long_source:
# "прошивка, ИЗМЕРЕНО (M4)"). В UTF-8. А subprocess с text=True декодирует
# вывод кодировкой ЛОКАЛИ: на Linux это UTF-8 и всё работает, на русской
# Windows — cp1251, и первый же русский байт роняет поток чтения внутри
# subprocess. Наружу это вылезает не как ошибка кодировки, а как
# stdout = None и потом «the JSON object must be str, not NoneType» —
# сообщение, по которому причину не найти.
#
# Поэтому кодировка задаётся ЯВНО во всех трёх вызовах. errors="replace":
# лучше получить один битый символ в сообщении об ошибке, чем потерять весь
# вывод.
_TXT = dict(capture_output=True, text=True, encoding="utf-8", errors="replace")

HERE = os.path.dirname(os.path.abspath(__file__))
HOSTCHECK = os.path.normpath(os.path.join(HERE, "..", "..", "hostcheck"))
_BIN = {}


def prebuilt(target="trace"):
    """Готовый бинарь рядом со скриптами — для машин без bash и g++.

    Windows: в папке bin/ лежит trace.exe, собранный из ТОГО ЖЕ
    gait_test.ino кросс-компилятором. Если его нет — собираем на месте.
    """
    env = os.environ.get("SPOTTY_%s_BIN" % target.upper())
    if env and os.path.exists(env):
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    # .exe кладётся для Windows; на linux его запускать нельзя, там собираем сами
    names = ((target + ".exe",) if os.name == "nt"
             else (target + ".bin", target + ".elf"))
    for name in names:
        cand = os.path.join(here, "..", "bin", name)
        if os.path.exists(cand):
            return os.path.normpath(cand)
    return None


def build(target="trace"):
    """Собрать hostcheck-цель и вернуть путь к бинарю. Собирается один раз."""
    if target in _BIN and os.path.exists(_BIN[target]):
        return _BIN[target]
    ready = prebuilt(target)
    if ready:
        _BIN[target] = ready
        return ready
    # Своя папка сборки на процесс: build.sh чистит её целиком, и два
    # параллельных работника затёрли бы друг друга.
    env = dict(os.environ, NORUN="1",
               TMPDIR=tempfile.mkdtemp(prefix="spotty_hc_"))
    r = subprocess.run(["bash", os.path.join(HOSTCHECK, "build.sh"), target],
                       env=env, **_TXT)
    if r.returncode != 0:
        raise RuntimeError("сборка %s провалилась:\n%s" % (target, r.stderr[-3000:]))
    path = r.stdout.strip().splitlines()[-1]
    if not os.path.exists(path):
        raise RuntimeError("бинарь не найден: %r\n%s" % (path, r.stdout))
    _BIN[target] = path
    return path


def _run(binary, args):
    r = subprocess.run([binary] + args, **_TXT)
    if r.returncode != 0:
        raise RuntimeError("прошивка вернула %d: %s"
                           % (r.returncode, (r.stderr or "")[-2000:]))
    if not r.stdout:
        # Пустой вывод при нулевом коде возврата — это не «стенд промолчал»,
        # а сорванное чтение потока. Говорим прямо, что запускали.
        raise RuntimeError(
            "стенд ничего не напечатал.\n  бинарь: %s\n  аргументы: %s\n"
            "  stderr: %s" % (binary, " ".join(args), (r.stderr or "")[-2000:]))
    try:
        return json.loads(r.stdout)
    except ValueError as exc:
        raise RuntimeError("стенд напечатал не JSON (%s):\n%s"
                           % (exc, r.stdout[:2000]))


def geometry(gait=0):
    """Геометрия звеньев, пределы каналов и фазы — из прошивки, не из головы.

    Фазы зависят от выбранной походки, поэтому gait здесь не декорация.
    """
    return _run(build("trace"), ["geom=1", "gait=%d" % gait])


def trace(params, out_csv=None, cycles=4.0, settle_ms=400.0, gait=0):
    """Прогнать цикл походки в прошивке и получить CSV углов.

    params — словарь параметров прошивки: height, span, lift, vel, swing,
    trim, penet, crab, shiftx, shifty, roll, turn, maxrate, engine.
    Возвращает (сводка, путь к CSV).
    """
    tmp = out_csv or tempfile.mktemp(prefix="spotty_traj_", suffix=".csv")
    args = ["gait=%d" % gait, "cycles=%.4f" % cycles, "settle=%.1f" % settle_ms,
            "out=" + tmp]
    for k, v in params.items():
        if isinstance(v, (list, tuple)):      # таблица сдвига по фазе: sx, sy
            args.append("%s=%s" % (k, ",".join("%.4f" % x for x in v)))
        else:
            args.append("%s=%.6f" % (k, float(v)))
    return _run(build("trace"), args), tmp


def ik_grid():
    """Сетка точек рабочей области с углами от настоящей legIK."""
    binary = build("ikdump")
    r = subprocess.run([binary], **_TXT)
    rows = []
    for line in r.stdout.splitlines()[1:]:
        rows.append([float(v) for v in line.split(",")])
    return rows
