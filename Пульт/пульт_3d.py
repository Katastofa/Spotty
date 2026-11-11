# -*- coding: utf-8 -*-
"""
ПУЛЬТ 3D — пульт управления Spotty с трёхмерной собакой на экране.

    ЗАПУСК_ПУЛЬТА.bat (в корне проекта) или Пульт\ПУЛЬТ.bat, либо:
    python пульт_3d.py                          робот spotty.local, пульт на http://127.0.0.1:8090/
    python пульт_3d.py --robot 192.168.1.42
    python пульт_3d.py --port 8091 --lan        открыть пульт и другим машинам в сети (осторожно)

Два режима на одной странице:

  ЖИВОЙ       собака на экране повторяет НАСТОЯЩИЕ углы 12 суставов, которые
              робот отдаёт в /api/state. Команды уходят в робота той же
              строкой, что с клавиатуры, через его POST /api/cmd.
  СИМУЛЯЦИЯ   робот не нужен. Траекторию суставов считает НАСТОЯЩАЯ прошивка
              через стенд hostcheck (trace / bin/trace.exe) — тот же путь,
              что у view.py и learn.py. Питоновской или JS-копии походки нет
              (SIMULATION.md R1).

ЧТО ЭТОТ СЕРВЕР ДЕЛАЕТ И ЧЕГО НЕ ДЕЛАЕТ
  * Прошивку не трогает, перепрошивка не нужна.
  * Сам робота НЕ опрашивает. Запрос к роботу уходит только тогда, когда его
    прислала открытая страница. Закрыли вкладку — опрос прекратился, и
    watchdog связи в прошивке (6 с -> 'x') работает как раньше. Если бы
    сервер опрашивал робота сам, он кормил бы watchdog вечно.
  * Команды принимает только POST и только со своей же страницы
    (Sec-Fetch-Site), как и сама прошивка. Иначе любая посторонняя вкладка
    в браузере оператора могла бы подать 'n'.
  * По умолчанию слушает только 127.0.0.1. С --lan пульт открыт всей сети:
    кто в сети, тот и управляет.
  * Кнопка СТОП в браузере — best effort по сети. Разрыв питания под рукой
    обязателен, SAFETY.md не смягчается.
"""
import argparse
import re
import socket
import csv
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Раскладка: Spotty/Пульт/ — пульт; Spotty/code/ — прошивка; Spotty/code/sim/ — симулятор
# (мост к прошивке spotty_sim и готовый bin/trace.exe берутся оттуда, копий нет).
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
CODE = os.path.join(ROOT, "code")
SIM = os.path.join(CODE, "sim")
sys.path.insert(0, SIM)
from spotty_sim import firmware  # noqa: E402  (мост к прошивке, без копии походки)
from spotty_sim import model as M  # noqa: E402  (генератор URDF, тот же, что у PyBullet)

PAGE = os.path.join(HERE, "пульт_3d.html")
ASM = os.path.join(HERE, "сборка_stl.json")          # куда какая деталь (сборка_stl.py)
STL_ZIP = os.path.join(ROOT, "SpotPupPi (Spotmini) - 3638679 - part 1 of 2.zip")

# Файлы, из которых собирается trace. Если любой новее бинаря — симуляция
# считает СТАРУЮ прошивку, и страница обязана это показать.
FW_SOURCES = [
    os.path.join(CODE, "gait_test", "gait_test.ino"),
    os.path.join(CODE, "gait_test", "oq_gait.h"),
    os.path.join(CODE, "gait_test", "spotty_config.h"),
    os.path.join(CODE, "spotty_config.h"),
    os.path.join(CODE, "spotty_osc.h"),
    os.path.join(CODE, "spotty_imu.h"),
    os.path.join(CODE, "gait_test", "spotty_tricks.h"),        # трюки (trace trick=N)
    os.path.join(CODE, "gait_test", "spotty_tricks_types.h"),
    os.path.join(CODE, "gait_test", "spotty_profiles.h"),       # профили ходьбы (M)
]

# Параметры, которые страница может передать в trace. Всё прочее отбрасывается.
TRACE_KEYS = {
    "engine", "height", "span", "lift", "penet", "vel", "swing", "crab",
    "shiftx", "shifty", "roll", "trim", "maxrate", "turn", "flat", "smooth",
    "shiftgain",
    "trick", "hipok", "hipsign",          # трюки (команда 'e'), 2026-09-29
    "amp", "knee", "duty", "period", "crouch", "hipamp",   # угловой движок: t k d T q a, 2026-09-30
}

CFG = {"robot": "spotty.local", "urdf": None, "stl_zip": STL_ZIP}
ROBOT_LOCK = threading.Lock()   # WebServer на ESP32 держит ОДНО соединение за раз
GEOM_CACHE = {}


# ---------------------------------------------------------------------------
#  прошивка (стенд)
# ---------------------------------------------------------------------------
def servo_channels():
    """Каналы PCA9685 по суставам — из code/spotty_config.h (SPOTTY_CH), не из головы."""
    import re
    try:
        with open(os.path.join(CODE, "spotty_config.h"), encoding="utf-8") as f:
            txt = f.read()
        m = re.search(r"SPOTTY_CH\s*\[[^\]]*\]\s*\[[^\]]*\]\s*=\s*\{(.*?)\};", txt, re.S)
        rows = re.findall(r"\{\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\}", m.group(1))
        if len(rows) != 4:
            return None
        return [int(x) for r in rows for x in r]        # FR, FL, BR, BL × хип, бедро, колено
    except Exception:
        return None


def servo_pwm():
    """Таблицы угол -> мкс из code/spotty_config.h: JOINT_ZERO_US, JOINT_SIGN,
    SPOTTY_US_PER_DEG. Нужны, чтобы в СИМУЛЯЦИИ показать импульс так же, как его
    считает spottyAngleToUsRaw. В ЖИВОМ импульс присылает сам робот (поле us)."""
    import re
    try:
        with open(os.path.join(CODE, "spotty_config.h"), encoding="utf-8") as f:
            txt = f.read()
        def arr(name):
            m = re.search(name + r"\s*\[16\]\s*=\s*\{(.*?)\};", txt, re.S)
            body = re.sub(r"//[^\n]*", "", m.group(1))
            v = [int(x) for x in re.findall(r"[-+]?\d+", body)]
            return v if len(v) == 16 else None
        upd = float(re.search(r"SPOTTY_US_PER_DEG\s*=\s*([\d.]+)", txt).group(1))
        z, sg = arr("JOINT_ZERO_US"), arr("JOINT_SIGN")
        lo, hi = arr("JOINT_MIN_US"), arr("JOINT_MAX_US")
        if not (z and sg):
            return None
        return {"zero": z, "sign": sg, "min": lo, "max": hi, "usPerDeg": upd}
    except Exception:
        return None


def firmware_info():
    """Какой бинарь считает траектории и не устарел ли он."""
    info = {"ok": False, "binary": None, "stale": False, "stale_files": [], "err": ""}
    try:
        path = firmware.build("trace")
    except Exception as exc:                      # нет g++/bash и нет trace.exe
        info["err"] = str(exc)[-600:]
        return info
    info["ok"] = True
    info["binary"] = path
    try:
        t_bin = os.path.getmtime(path)
        for src in FW_SOURCES:
            if os.path.exists(src) and os.path.getmtime(src) > t_bin + 1:
                info["stale_files"].append(os.path.relpath(src, CODE))
        info["stale"] = bool(info["stale_files"])
        info["binary_date"] = time.strftime("%Y-%m-%d %H:%M", time.localtime(t_bin))
    except OSError:
        pass
    return info


def geometry(gait):
    gait = int(gait)
    if gait not in GEOM_CACHE:
        GEOM_CACHE[gait] = firmware.geometry(gait)
    return GEOM_CACHE[gait]


def urdf_text(gait=0):
    """URDF собаки — тем же генератором, что и для PyBullet (evaluate.prepare_urdf),
    из геометрии и пределов ПРОШИВКИ. Файл на диск не пишется: spotty_gen.urdf
    принадлежит симулятору, пульт его не трогает."""
    if CFG["urdf"]:                               # --urdf: взять готовый файл как есть
        with open(CFG["urdf"], encoding="utf-8") as f:
            return f.read()
    geom = geometry(gait)
    m = M.Model().with_geometry(geom["geom_mm"])
    L = geom["limits_deg"]
    limits = [[L[3 * i], L[3 * i + 1], L[3 * i + 2]] for i in range(4)]
    return M.build_urdf(m, limits_deg=limits)


def mesh_assembly():
    """Сборка внешнего вида + проверка, что архив STL на месте."""
    with open(ASM, encoding="utf-8") as f:
        asm = json.load(f)
    asm["zip_ok"] = os.path.exists(CFG["stl_zip"])
    asm["zip_path"] = CFG["stl_zip"]
    return asm


_ZIP = {}
def mesh_stl(name):
    """Одна деталь STL прямо из архива, без распаковки. Только из списка сборки."""
    allowed = {p["file"] for p in mesh_assembly()["parts"]}
    if name not in allowed:
        raise KeyError("детали нет в сборке: %s" % name)
    path = CFG["stl_zip"]
    if path not in _ZIP:
        _ZIP[path] = zipfile.ZipFile(path)
    asm_dir = mesh_assembly().get("zip_dir", "files/")
    return _ZIP[path].read(asm_dir + name)


def run_trace(req):
    gait = int(req.get("gait", 0))
    if not 0 <= gait <= 4:
        raise ValueError("gait вне 0..4")
    cycles = float(req.get("cycles", 1))
    if not 0 <= cycles <= 20:
        raise ValueError("cycles вне 0..20")
    settle = float(req.get("settle", 0))
    params = {}
    for k, v in (req.get("params") or {}).items():
        if k in TRACE_KEYS:
            params[k] = float(v)
    summary, csv_path = firmware.trace(params, cycles=cycles, settle_ms=settle, gait=gait)
    step = max(1, int(req.get("decim", 3)))
    frames, phase, t = [], [], []
    try:
        with open(csv_path, newline="", encoding="utf-8") as f:
            rd = csv.reader(f)
            next(rd, None)
            for i, row in enumerate(rd):
                if i % step:
                    continue
                t.append(round(float(row[0]), 4))
                frames.append([round(float(x), 2) for x in row[1:13]])
                phase.append(round(float(row[13]), 3))
    finally:
        try:
            os.remove(csv_path)
        except OSError:
            pass
    summary.pop("csv", None)
    summary["dt_s"] = (summary.get("tick_ms", 3) * step) / 1000.0
    if "key_rows" in summary:            # начала кадров трюка, с — для полоски прогресса
        summary["key_t"] = [r * summary.get("tick_ms", 3) / 1000.0 for r in summary["key_rows"]]
    summary["t"] = t
    summary["frames"] = frames
    summary["phase_col"] = phase
    return summary


# ---------------------------------------------------------------------------
#  робот
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# АДРЕС РОБОТА: имя -> IP один раз (2026-09-29).
# Windows ищет «spotty.local» через mDNS медленно и не всегда: каждый опрос
# по имени мог стоить секунду-две или падать, и пульт мигал «НЕТ СВЯЗИ».
# После первого удачного ответа имя разрешается в IP, и дальше ходим по IP.
# IP перестал отвечать (роутер выдал новый) — забываем и снова идём по имени.
# ---------------------------------------------------------------------------
_IP_RX = re.compile(r"^\d+\.\d+\.\d+\.\d+(:\d+)?$")


def _host():
    return CFG.get("ip") or CFG["robot"]


def _remember_ip():
    if CFG.get("ip") or _IP_RX.match(CFG["robot"]):
        return
    host, _, port = CFG["robot"].partition(":")
    try:
        ip = socket.gethostbyname(host)
        CFG["ip"] = ip + (":" + port if port else "")
        print("  робот %s = %s, дальше по IP" % (CFG["robot"], CFG["ip"]), flush=True)
    except OSError:
        pass


def _forget_ip():
    if CFG.get("ip"):
        print("  IP %s не отвечает, снова по имени %s" % (CFG["ip"], CFG["robot"]), flush=True)
        CFG["ip"] = None


# Своя сеть робота (SPOTTY_WIFI_MODE, spotty_wifi.h) — адрес всегда этот.
# Ноутбук подключён к «Spotty-Reboot», а в пульте осталось «spotty.local» —
# пульт сам попробует его, не заставляя менять адрес руками.
AP_ADDR = "192.168.4.1"
_AP_TRY = {"at": 0.0}


def robot_get(path, timeout=2.5):
    url = "http://%s%s" % (_host(), path)
    with ROBOT_LOCK:
        try:
            with urllib.request.urlopen(url, timeout=timeout) as r:
                out = r.status, r.read()
        except urllib.error.HTTPError:
            raise
        except Exception:
            had_ip = bool(CFG.get("ip"))
            _forget_ip()
            now = time.time()
            if (not had_ip and CFG["robot"] != AP_ADDR and not _IP_RX.match(CFG["robot"])
                    and now - _AP_TRY["at"] > 10):
                _AP_TRY["at"] = now
                try:
                    with urllib.request.urlopen("http://%s%s" % (AP_ADDR, path), timeout=1.0) as r:
                        out = r.status, r.read()
                    CFG["ip"] = AP_ADDR
                    print("  робот найден в своей сети: %s" % AP_ADDR, flush=True)
                    return out
                except Exception:
                    pass
            raise
    _remember_ip()
    return out


def robot_cmd(line, timeout=3.0):
    data = urllib.parse.urlencode({"c": line}).encode()
    url = "http://%s/api/cmd" % _host()
    req = urllib.request.Request(url, data=data, method="POST")
    with ROBOT_LOCK:
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.status, r.read()
        except urllib.error.HTTPError as e:
            return e.code, e.read()
        except Exception:
            _forget_ip()
            raise


# ---------------------------------------------------------------------------
#  HTTP
# ---------------------------------------------------------------------------
class H(BaseHTTPRequestHandler):
    server_version = "spotty-pult3d"

    def log_message(self, fmt, *a):          # консоль не засоряем опросом
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code, obj):
        self._send(code, json.dumps(obj, ensure_ascii=False))

    def _same_origin(self):
        site = self.headers.get("Sec-Fetch-Site", "")
        return site in ("", "same-origin")

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n > 65536:
            raise ValueError("слишком большой запрос")
        return self.rfile.read(n) if n else b""

    # ---------------- GET ----------------
    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        if u.path in ("/", "/index.html"):
            try:
                with open(PAGE, "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            except OSError:
                self._send(500, "нет файла %s" % PAGE, "text/plain; charset=utf-8")
            return
        if u.path == "/pult/info":
            self._json(200, {"robot": CFG["robot"], "robot_ip": CFG.get("ip"), "fw": firmware_info(), "ch": servo_channels(), "pwm": servo_pwm()})
            return
        if u.path == "/sim/geom":
            q = urllib.parse.parse_qs(u.query)
            try:
                self._json(200, geometry(q.get("gait", ["0"])[0]))
            except Exception as exc:
                self._json(500, {"err": str(exc)[-800:]})
            return
        if u.path == "/mesh/assembly":
            try:
                self._json(200, mesh_assembly())
            except Exception as exc:
                self._json(500, {"err": str(exc)[-800:]})
            return
        if u.path == "/mesh/stl":
            q = urllib.parse.parse_qs(u.query)
            try:
                data = mesh_stl((q.get("f") or [""])[0])
                self.send_response(200)
                self.send_header("Content-Type", "model/stl")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "max-age=3600")
                self.end_headers()
                self.wfile.write(data)
            except Exception as exc:
                self._json(404, {"err": str(exc)[-400:]})
            return
        if u.path == "/sim/urdf":
            try:
                self._send(200, urdf_text(), "application/xml; charset=utf-8")
            except Exception as exc:
                self._json(500, {"err": str(exc)[-800:]})
            return
        if u.path in ("/api/state", "/api/log"):
            try:
                code, body = robot_get(self.path)
                self._send(code, body)
            except Exception as exc:
                self._json(502, {"err": "робот %s не отвечает: %s" % (CFG["robot"], exc)})
            return
        self._send(404, "нет такого", "text/plain; charset=utf-8")

    # ---------------- POST ---------------
    def do_POST(self):
        u = urllib.parse.urlparse(self.path)
        if not self._same_origin():
            self._send(403, "cross-site", "text/plain; charset=utf-8")
            return
        try:
            body = self._body()
        except ValueError as exc:
            self._send(413, str(exc), "text/plain; charset=utf-8")
            return

        if u.path == "/api/cmd":
            q = urllib.parse.parse_qs(u.query)
            if not q.get("c") and body:
                q = urllib.parse.parse_qs(body.decode("utf-8", "replace"))
            line = (q.get("c") or [""])[0].strip()
            if not line:
                self._send(400, "empty", "text/plain; charset=utf-8")
                return
            # СТОП повторяется: потерянный пакет не должен стоить остановки.
            # Остальные команды — нет: ответ мог потеряться ПОСЛЕ исполнения,
            # и повтор 'w' или трюка выполнил бы его дважды.
            tries = 3 if line == "x" else 1
            for k in range(tries):
                try:
                    code, resp = robot_cmd(line)
                    self._send(code, resp, "text/plain; charset=utf-8")
                    break
                except Exception as exc:
                    if k == tries - 1:
                        self._send(502, "робот не отвечает: %s" % exc, "text/plain; charset=utf-8")
                    else:
                        time.sleep(0.15)
            return

        if u.path == "/sim/trace":
            try:
                req = json.loads(body.decode("utf-8") or "{}")
                self._json(200, run_trace(req))
            except Exception as exc:
                self._json(500, {"err": str(exc)[-1500:]})
            return

        if u.path == "/pult/robot":
            try:
                req = json.loads(body.decode("utf-8") or "{}")
                addr = str(req.get("robot", "")).strip()
                ok = addr and len(addr) < 80 and all(
                    c.isalnum() or c in ".-:_" for c in addr)
                if not ok:
                    raise ValueError("адрес: буквы, цифры, точка, дефис, двоеточие")
                CFG["robot"] = addr
                CFG["ip"] = None
                self._json(200, {"robot": addr})
            except Exception as exc:
                self._json(400, {"err": str(exc)})
            return

        self._send(404, "нет такого", "text/plain; charset=utf-8")


def main():
    ap = argparse.ArgumentParser(description="Пульт Spotty с 3D-собакой")
    ap.add_argument("--robot", default="spotty.local", help="адрес робота (IP или spotty.local)")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--lan", action="store_true",
                    help="слушать всю сеть, а не только этот компьютер")
    ap.add_argument("--urdf", default=None,
                    help="нарисовать собаку из этого URDF (по умолчанию — генератор "
                         "model.py из геометрии прошивки, как у PyBullet)")
    ap.add_argument("--stl-zip", default=STL_ZIP,
                    help="архив STL SpotPupPi для внешнего вида собаки")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    CFG["robot"] = a.robot
    CFG["urdf"] = os.path.abspath(a.urdf) if a.urdf else None
    CFG["stl_zip"] = os.path.abspath(a.stl_zip)

    fw = firmware_info()
    print("Spotty — пульт 3D")
    print("  робот:      http://%s/" % a.robot)
    if fw["ok"]:
        print("  прошивка:   %s%s" % (fw["binary"], "  (собран %s)" % fw.get("binary_date", "")))
        if fw["stale"]:
            print("  [!] trace СТАРЕЕ исходников: %s" % ", ".join(fw["stale_files"]))
            print("      симуляция покажет прежнюю прошивку. Пересобрать:")
            print("      hostcheck/crossbuild.sh trace ../sim/bin")
    else:
        print("  [!] стенд не собран, СИМУЛЯЦИЯ недоступна: %s" % fw["err"][:300])

    print("  модель:     %s" % (CFG["urdf"] or "URDF из model.py по геометрии прошивки"))
    print("  детали:     %s" % (CFG["stl_zip"] if os.path.exists(CFG["stl_zip"])
                                  else "архив STL не найден — собака будет схемой"))
    host = "0.0.0.0" if a.lan else "127.0.0.1"
    srv = ThreadingHTTPServer((host, a.port), H)
    url = "http://127.0.0.1:%d/" % a.port
    print("  пульт:      %s%s" % (url, "   (открыт ВСЕЙ сети: кто в сети — тот и управляет)" if a.lan else ""))
    print("  выход:      Ctrl+C")
    if not a.no_browser:
        try:
            import webbrowser
            threading.Timer(0.8, lambda: webbrowser.open(url)).start()
        except Exception:
            pass
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nпульт закрыт")


if __name__ == "__main__":
    main()
