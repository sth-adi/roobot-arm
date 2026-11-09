"""Try the UI -> AI path in a browser with no arm:  python sim/front_demo.py   then open http://127.0.0.1:8090
Serves a tiny test page and the placeholder endpoints of dashboard/interface.handle_http (POST /request, GET /job, GET /state, POST /stop) on top of the
virtual arm (sim/dry_run.VirtualArm, planner-checked, instant). With GROQ_API_KEY set the AI chooses; without it the keyword matcher does.
Add --stall to make the first 'down to the key' stall once and watch the recovery. Local only (127.0.0.1)."""
import argparse
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "dashboard"))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import dry_run as DR  # noqa: E402
import interface as I  # noqa: E402

PAGE = """<!doctype html><meta charset=utf-8><title>UI to AI test page</title>
<style>body{font:15px system-ui;max-width:640px;margin:2rem auto;padding:0 1rem}input{width:70%%;padding:.4rem}button{padding:.4rem .7rem;margin:.15rem}
pre{background:#f4f4f4;padding:.6rem;overflow:auto;max-height:14rem}#reply{font-weight:600;margin:.6rem 0}</style>
<h2>UI to AI test page</h2><p>Virtual arm only. Type or speak a request; the AI picks one action from the menu.</p>
<input id=t placeholder="get me the second key" autofocus> <button onclick=ask()>Send</button><br>
<button onclick="btn('fetch',1)">Fetch 1</button><button onclick="btn('fetch',2)">Fetch 2</button><button onclick="btn('fetch',3)">Fetch 3</button>
<button onclick="btn('return',1)">Return 1</button><button onclick="btn('return',2)">Return 2</button><button onclick="btn('return',3)">Return 3</button>
<button onclick="btn('home')">Home</button> <button onclick="btn('stop')" style="color:#b00">STOP</button>
<div id=reply></div><div id=ctl></div><h4>Job</h4><pre id=job>-</pre><h4>Last reply (raw)</h4><pre id=raw>-</pre>
<script>
let tok=null;
async function post(p,b){const r=await fetch(p,{method:'POST',body:JSON.stringify(b)});return r.json()}
function show(r){document.getElementById('raw').textContent=JSON.stringify(r,null,1);document.getElementById('reply').textContent=r.message||'';
 tok=r.kind==='confirm'?r.token:null;document.getElementById('ctl').innerHTML=tok?'<button onclick=yes()>Yes</button><button onclick=no()>No</button>':''}
async function ask(){const t=document.getElementById('t');show(await post('/request',{v:1,id:String(Date.now()),text:t.value}));t.value=''}
async function btn(a,k){show(await post('/request',{v:1,id:String(Date.now()),action:a,key:k}))}
async function yes(){show(await post('/request',{confirm:tok}))}
async function no(){show(await post('/request',{cancel:tok}))}
document.getElementById('t').addEventListener('keydown',e=>{if(e.key==='Enter')ask()});
setInterval(async()=>{const r=await fetch('/job');document.getElementById('job').textContent=r.ok?JSON.stringify(await r.json(),null,1):'no job yet'},700);
</script>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--stall", action="store_true", help="the first 'down to the key' stalls once")
    a = ap.parse_args()
    arm = DR.VirtualArm("down to the key" if a.stall else None)
    front = I.Frontdoor(DR.make_store(), arm, lambda: dict(arm.pose))

    class H(BaseHTTPRequestHandler):
        def _send(self, code, body, ctype="application/json"):
            import json
            data = body.encode() if isinstance(body, str) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                return self._send(200, PAGE.replace("%%", "%"), "text/html; charset=utf-8")
            self._send(*I.handle_http(front, "GET", self.path))

        def do_POST(self):
            n = min(int(self.headers.get("Content-Length") or 0), 8192)
            self._send(*I.handle_http(front, "POST", self.path, self.rfile.read(n)))

        def log_message(self, *args):
            pass

    print("UI to AI test page on http://127.0.0.1:%d (virtual arm%s)" % (a.port, ", first grab will stall" if a.stall else ""))
    ThreadingHTTPServer(("127.0.0.1", a.port), H).serve_forever()


if __name__ == "__main__":
    main()
