# -*- coding: utf-8 -*-
"""
ВЫСТАВОЧНЫЙ ПУЛЬТ — страница с крупными кнопками для посетителей.

    python пульт_выставка.py --robot 192.168.1.42
    python пульт_выставка.py --robot spotty.local --port 8080

Посетитель открывает http://<ip ноутбука>:8080/ на телефоне и нажимает
кнопку. Пульт НЕ передаёт в робота то, что пришло из браузера: у каждой
кнопки есть номер, а строки команд лежат здесь, в ACTIONS. Что бы ни
прислал браузер, в робота уйдёт только одна из этих строк.

ПОЧЕМУ ОТДЕЛЬНАЯ ПРОГРАММА, А НЕ ПРАВКА ПРОШИВКИ. До главной выставки
прошивку лучше не трогать: любая правка требует перепрошивки и проверки
на железе. Пульт живёт на ноутбуке, общается с роботом его же
веб-интерфейсом (POST /api/cmd) и в любой момент закрывается по Ctrl+C.

ЧТО ОСТАЁТСЯ ЗА ОПЕРАТОРОМ:
  * взвод ('n') и остановку ('x', 'r') посетителям не отдаём — это в
    экране оператора (клавиша O) и по USB;
  * пока стоит ЗАМОК, кнопки посетителей не исполняются;
  * пульт сам делает паузу между показами: сервоприводам нужен отдых.

Никакие команды, кроме перечисленных в ACTIONS, через этот пульт
невозможны. Строки команд — те же, что оператор вводит с клавиатуры.
"""
import argparse
import html
import json
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ---------------------------------------------------------------------------
# ЧТО МОЖЕТ ПОСЕТИТЕЛЬ. Строки — команды прошивки, как с клавиатуры.
# cooldown — сколько секунд кнопка не работает после нажатия (отдых серво).
# ВНИМАНИЕ: сюда нельзя вписывать 'n' (взвод) — взводит только оператор.
# ---------------------------------------------------------------------------
ACTIONS = [
    {"id": "walk3",  "title": "Пройти",   "sub": "три цикла шага",
     "cmds": ["c 3", "w"], "cooldown": 25, "hold": 8, "on": True},
    {"id": "walk1",  "title": "Один шаг", "sub": "один цикл — видно, как работает нога",
     "cmds": ["c 1", "w"], "cooldown": 15, "hold": 4, "on": True},
    # РАЗВОРОТ ВЫКЛЮЧЕН по умолчанию: команда 'u' на железе ещё не проверена
    # в декартовом движке. Проверьте её сами на вывешенном роботе, и если
    # разворот выходит аккуратным — поставьте "on": True.
    {"id": "turn",   "title": "Развернуться", "sub": "поворот на месте",
     "cmds": ["u 1", "c 2", "w", "u 0"], "cooldown": 30, "hold": 10, "on": False},
]
ACTIONS = [a for a in ACTIONS if a.get("on")]

# Команда, которой пульт останавливает робота (кнопка оператора).
STOP_CMD = "x"

STATE = {
    "locked": True,          # ЗАМОК: по умолчанию кнопки посетителей молчат
    "busy_until": 0.0,       # робот занят показом до этого времени
    "cooldowns": {},         # id -> время, до которого кнопка спит
    "last_error": "",
    "count": 0,              # сколько показов за сессию
}
LOCK = threading.Lock()


# ---------------------------------------------------------------------------
def robot_post(robot, line, timeout=3.0):
    """Отправить одну строку команды в робота. Возвращает (ok, текст)."""
    data = urllib.parse.urlencode({"c": line}).encode()
    url = "http://%s/api/cmd" % robot
    req = urllib.request.Request(url, data=data, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return True, r.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as e:
        return False, "робот ответил %d" % e.code
    except Exception as e:                      # сеть, таймаут, имя не найдено
        return False, str(e)


def robot_state(robot, timeout=2.0):
    """GET /api/state — заодно кормит watchdog робота (любой запрос считается)."""
    try:
        with urllib.request.urlopen("http://%s/api/state" % robot, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8", "replace"))
    except Exception:
        return None


class Keeper(threading.Thread):
    """Пока идёт показ — стучимся в робота, чтобы он не посчитал пульт потерянным.

    Watchdog прошивки останавливает робота, если браузер замолчал. Наши
    запросы идут с ноутбука, поэтому молчания быть не должно ровно столько,
    сколько длится показ.
    """
    daemon = True

    def __init__(self, robot):
        super().__init__()
        self.robot = robot

    def run(self):
        while True:
            with LOCK:
                busy = time.time() < STATE["busy_until"]
            if busy:
                robot_state(self.robot)
            time.sleep(0.7)


# ---------------------------------------------------------------------------
PAGE = u"""<!doctype html><html lang="ru"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Spotty — пульт</title>
<style>
 :root{color-scheme:dark}
 *{box-sizing:border-box}
 body{margin:0;background:#0c121c;color:#eaf0f7;
      font:16px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
      padding:20px 16px 40px;max-width:560px;margin:0 auto}
 h1{font-size:28px;margin:8px 0 2px}
 .sub{color:#8fa6bf;margin-bottom:22px}
 button.act{display:block;width:100%;margin:0 0 14px;padding:22px 20px;
   border:0;border-radius:18px;background:#1b62d6;color:#fff;text-align:left;
   font-size:22px;font-weight:600;cursor:pointer}
 button.act small{display:block;font-size:15px;font-weight:400;opacity:.8;margin-top:4px}
 button.act:disabled{background:#26303e;color:#7b8798}
 #msg{margin:18px 0;padding:14px 16px;border-radius:14px;background:#16202e;
   color:#b8c8dc;min-height:24px}
 .warn{color:#ffb454}
 footer{margin-top:28px;color:#63748a;font-size:13px}
</style></head><body>
<h1>Spotty</h1>
<div class="sub">Четвероногий робот · ESP32-S3 · 12 сервоприводов</div>
__BUTTONS__
<div id="msg">Нажмите кнопку — робот выполнит одно движение.</div>
<footer>Робот двигается только под присмотром оператора.
Между показами нужна пауза: сервоприводы греются.</footer>
<script>
var busy=false;
function poll(){
  fetch('/api/kiosk',{cache:'no-store'}).then(function(r){return r.json()}).then(function(s){
    var now=Date.now()/1000;
    document.querySelectorAll('button.act').forEach(function(b){
      var left=Math.max(0,(s.cooldowns[b.dataset.id]||0)-now, s.busy_left||0);
      b.disabled=s.locked||left>0.4;
      var base=b.dataset.title;
      b.firstChild.nodeValue = left>0.4 ? base+' · '+Math.ceil(left)+' с' : base;
    });
    var m=document.getElementById('msg');
    if(s.locked){m.innerHTML='<span class="warn">Оператор пока закрыл управление.</span>';}
    else if(s.busy_left>0.4){m.textContent='Робот выполняет движение…';}
    else if(s.last_error){m.innerHTML='<span class="warn">'+s.last_error+'</span>';}
    else if(!busy){m.textContent='Нажмите кнопку — робот выполнит одно движение.';}
  }).catch(function(){});
}
function go(id){
  busy=true;
  document.getElementById('msg').textContent='Отправляю…';
  fetch('/api/do',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({id:id})})
    .then(function(r){return r.json()})
    .then(function(s){
      busy=false;
      document.getElementById('msg').textContent = s.ok ? 'Готово, смотрите на робота.'
        : (s.error||'Не получилось');
      poll();
    }).catch(function(){busy=false;});
}
setInterval(poll,700); poll();
</script></body></html>"""

OPER = u"""<!doctype html><html lang="ru"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Пульт — оператор</title>
<style>
 :root{color-scheme:dark}
 body{margin:0;background:#0c121c;color:#eaf0f7;font:16px system-ui,sans-serif;
      padding:24px;max-width:560px;margin:0 auto}
 button{width:100%;padding:20px;margin-bottom:14px;border:0;border-radius:16px;
        font-size:20px;font-weight:600;cursor:pointer;color:#fff}
 .open{background:#1f8a4c}.close{background:#8a5a1f}.stop{background:#b3241f}
 #st{padding:14px;border-radius:14px;background:#16202e;color:#b8c8dc;margin-bottom:20px}
</style></head><body>
<h2>Экран оператора</h2>
<div id="st">…</div>
<button class="open"  onclick="send('unlock')">ОТКРЫТЬ кнопки посетителям</button>
<button class="close" onclick="send('lock')">ЗАКРЫТЬ кнопки</button>
<button class="stop"  onclick="send('stop')">СТОП роботу</button>
<p style="color:#63748a">Взвод ('n'), снятие питания ('r') и настройка походки —
только с ноутбука по USB. Здесь их нет намеренно.</p>
<script>
function upd(){fetch('/api/kiosk',{cache:'no-store'}).then(r=>r.json()).then(s=>{
  document.getElementById('st').textContent =
    (s.locked?'ЗАМОК стоит — посетители не могут ничего запустить':'Кнопки ОТКРЫТЫ')
    + ' · показов за сессию: ' + s.count + (s.last_error? ' · ' + s.last_error : '');
});}
function send(a){fetch('/api/oper',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({a:a})}).then(upd);}
setInterval(upd,1000);upd();
</script></body></html>"""


def build_page():
    btns = []
    for a in ACTIONS:
        btns.append(
            u'<button class="act" data-id="%s" data-title="%s" onclick="go(\'%s\')">%s'
            u'<small>%s</small></button>'
            % (html.escape(a["id"]), html.escape(a["title"]), html.escape(a["id"]),
               html.escape(a["title"]), html.escape(a["sub"])))
    return PAGE.replace("__BUTTONS__", u"\n".join(btns))


# ---------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    robot = "spotty.local"
    oper_path = "/operator"
    page = ""

    def log_message(self, fmt, *args):            # тихий лог: одна строка на действие
        pass

    def _send(self, code, body, ctype="text/html; charset=utf-8"):
        data = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj, ensure_ascii=False), "application/json; charset=utf-8")

    def do_GET(self):
        path = urllib.parse.urlparse(self.path).path
        if path == "/":
            self._send(200, self.page)
        elif path == self.oper_path:
            self._send(200, OPER)
        elif path == "/api/kiosk":
            now = time.time()
            with LOCK:
                self._json({
                    "locked": STATE["locked"],
                    "busy_left": max(0.0, STATE["busy_until"] - now),
                    "cooldowns": STATE["cooldowns"],
                    "last_error": STATE["last_error"],
                    "count": STATE["count"],
                })
        else:
            self._send(404, "нет такой страницы")

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0 or n > 4096:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8", "replace"))
        except ValueError:
            return {}

    def do_POST(self):
        path = urllib.parse.urlparse(self.path).path
        if path == "/api/do":
            return self._visitor(self._body())
        if path == "/api/oper":
            return self._operator(self._body())
        self._send(404, "нет такой страницы")

    # --- посетитель --------------------------------------------------------
    def _visitor(self, body):
        # Идентификатор кнопки ищется в СВОЁМ списке. Ничего из тела запроса
        # в робота не попадает — только строки из ACTIONS.
        act = next((a for a in ACTIONS if a["id"] == body.get("id")), None)
        if act is None:
            return self._json({"ok": False, "error": "неизвестная кнопка"}, 400)
        now = time.time()
        with LOCK:
            if STATE["locked"]:
                return self._json({"ok": False, "error": "оператор закрыл управление"}, 403)
            if now < STATE["busy_until"]:
                return self._json({"ok": False, "error": "робот ещё выполняет движение"}, 429)
            if now < STATE["cooldowns"].get(act["id"], 0):
                return self._json({"ok": False, "error": "кнопка отдыхает"}, 429)
            STATE["busy_until"] = now + act["hold"]
            STATE["cooldowns"][act["id"]] = now + act["cooldown"]
        for line in act["cmds"]:
            ok, msg = robot_post(self.robot, line)
            if not ok:
                with LOCK:
                    STATE["busy_until"] = 0.0
                    STATE["last_error"] = u"робот не отвечает: %s" % msg
                print(u"[!] %s -> %s" % (line, msg), flush=True)
                return self._json({"ok": False, "error": u"робот не отвечает"}, 502)
            time.sleep(0.05)
        with LOCK:
            STATE["last_error"] = ""
            STATE["count"] += 1
            n = STATE["count"]
        print(u"[%s] показ %d: %s" % (time.strftime("%H:%M:%S"), n,
                                      " ; ".join(act["cmds"])), flush=True)
        return self._json({"ok": True})

    # --- оператор ----------------------------------------------------------
    def _operator(self, body):
        a = body.get("a")
        if a == "lock":
            with LOCK:
                STATE["locked"] = True
            print("[оператор] замок", flush=True)
        elif a == "unlock":
            with LOCK:
                STATE["locked"] = False
                STATE["last_error"] = ""
            print("[оператор] кнопки открыты", flush=True)
        elif a == "stop":
            ok, msg = robot_post(self.robot, STOP_CMD)
            with LOCK:
                STATE["locked"] = True
                STATE["busy_until"] = 0.0
                STATE["last_error"] = "" if ok else u"стоп не прошёл: %s" % msg
            print("[оператор] СТОП -> %s" % ("ok" if ok else msg), flush=True)
        else:
            return self._json({"ok": False}, 400)
        return self._json({"ok": True})


def local_ips():
    import socket
    out = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        out.append(s.getsockname()[0])
        s.close()
    except Exception:
        pass
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default=os.environ.get("SPOTTY_ROBOT", "spotty.local"),
                    help="адрес робота: IP из Serial или spotty.local")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--oper-path", default="/operator",
                    help="адрес экрана оператора (не показывайте его посетителям)")
    args = ap.parse_args()

    Handler.robot = args.robot
    Handler.oper_path = args.oper_path
    Handler.page = build_page()
    Keeper(args.robot).start()

    st = robot_state(args.robot)
    print(u"робот %s — %s" % (args.robot,
                              u"отвечает" if st else u"НЕ отвечает (проверьте адрес и сеть)"))
    for ip in local_ips():
        print(u"  посетителям:  http://%s:%d/" % (ip, args.port))
        print(u"  оператору:    http://%s:%d%s" % (ip, args.port, args.oper_path))
    print(u"Замок стоит: откройте кнопки на экране оператора, когда робот взведён.")
    print(u"Ctrl+C — закрыть пульт.")
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nпульт закрыт")


if __name__ == "__main__":
    main()
