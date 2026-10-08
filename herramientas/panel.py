# panel.py — PANEL del equipo en el navegador (solo Python, sin instalar nada).
#
# Muestra en vivo la cancha tal como la publica la cámara (o el simulador):
# rovers, cubos, zonas, fase y reloj, latencia... y, si los robots tienen
# DEPURACION_HOST apuntando a esta compu, qué está pensando cada uno (estado,
# cubo asignado, ruta, camino reservado).
#
# Con MODO_PRUEBAS = True en el robot, también se le pueden dar órdenes de
# prueba (ir a un punto haciendo clic, girar, agarrar o llevar un cubo, ir
# recto para medir la velocidad). Eso es SOLO para probar en casa: en la
# competencia los robots no escuchan órdenes (reglamento 6.3 y 11.2).
#
#   python herramientas/panel.py                       # visión/simulador en esta compu
#   python herramientas/panel.py --vision 192.168.1.47 # visión en otra compu
# y abrir http://localhost:8080

import argparse
import json
import os
import socket
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

REG = {"f": None, "ult_tele": 0, "ult_flush": 0}


def registrar(tipo, dato):
    """Guarda todo lo que pasa en registros/sesion_*.ndjson (para revisar después)."""
    f = REG["f"]
    if f is None:
        return
    ahora = int(time.time() * 1000)
    try:
        f.write(json.dumps({"t": ahora, "k": tipo, "d": dato}) + "\n")
        if ahora - REG["ult_flush"] > 1000:
            f.flush()
            REG["ult_flush"] = ahora
    except (OSError, ValueError):
        pass


ESTADO = {"tele": None, "rx_ms": 0, "lat": [], "saltos": 0, "seq": None, "n": 0,
          "conectado": False, "robots": {}, "ips": {}}
CANDADO = threading.Lock()
UDP = None


RELEVO = {"ult": 0}


def reenviar(linea, ahora):
    """Reenvía la telemetría a los robots por UDP, 10 veces por segundo (solo
    para probar en casa: el robot la usa si su conexión TCP con la visión se
    traba, que con un WiFi flojo pasa seguido)."""
    if UDP is None or ahora - RELEVO["ult"] < 100 or len(linea) > 1090:
        return
    RELEVO["ult"] = ahora
    with CANDADO:
        dest = [(ESTADO["ips"][k][0]) for k, r in ESTADO["robots"].items()
                if k in ESTADO["ips"] and ahora - r.get("_ms", 0) < 5000]
    for ip in dest:
        try:
            UDP.sendto(linea + b"\n", (ip, 2027))
        except OSError:
            pass


def hilo_vision(host, port):
    while True:
        try:
            s = socket.create_connection((host, port), timeout=3)
            s.settimeout(3)
            with CANDADO:
                ESTADO["conectado"] = True
            buf = b""
            while True:
                d = s.recv(65536)
                if not d:
                    break
                buf += d
                if b"\n" not in buf:
                    continue
                partes = buf.split(b"\n")
                buf = partes[-1]
                try:
                    m = json.loads(partes[-2])
                except ValueError:
                    continue
                ahora = int(time.time() * 1000)
                reenviar(partes[-2], ahora)
                with CANDADO:
                    if ESTADO["seq"] is not None and m.get("seq", 0) > ESTADO["seq"] + 1:
                        ESTADO["saltos"] += 1
                    ESTADO["seq"] = m.get("seq")
                    ESTADO["tele"] = m
                    ESTADO["rx_ms"] = ahora
                    ESTADO["n"] += 1
                    ts = m.get("ts_ms", ahora)
                    if ts != ESTADO.get("ts_ult"):
                        # foto nueva: cuánto tardó en llegar y cada cuánto llegan
                        ESTADO["ts_ult"] = ts
                        ESTADO["lat"] = (ESTADO["lat"] + [ahora - ts])[-40:]
                        ESTADO["fotos"] = (ESTADO.get("fotos", []) + [ahora])[-30:]
                    if ahora - REG["ult_tele"] >= 100:          # 10 por segundo alcanza
                        REG["ult_tele"] = ahora
                        registrar("tele", m)
        except OSError:
            pass
        with CANDADO:
            ESTADO["conectado"] = False
        time.sleep(1)


def hilo_udp():
    while True:
        try:
            d, origen = UDP.recvfrom(4096)
            r = json.loads(d.decode())
            with CANDADO:
                r["_ms"] = int(time.time() * 1000)
                registrar("robot", r)
                if r.get("e") == "SIN_VISION":     # informe corto: conservo lo demás
                    viejo = dict(ESTADO["robots"].get(str(r.get("id"))) or {})
                    viejo.update(r)
                    viejo["m"] = "SIN VISIÓN: no recibe datos de la cámara"
                    r = viejo
                ESTADO["robots"][str(r.get("id"))] = r
                ESTADO["ips"][str(r.get("id"))] = origen
        except (OSError, ValueError):
            pass


class Pagina(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _resp(self, cuerpo, tipo):
        b = cuerpo.encode() if isinstance(cuerpo, str) else cuerpo
        self.send_response(200)
        self.send_header("Content-Type", tipo)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path.startswith("/estado"):
            with CANDADO:
                lat = sorted(ESTADO["lat"])
                d = {"tele": ESTADO["tele"], "conectado": ESTADO["conectado"],
                     "edad": int(time.time() * 1000) - ESTADO["rx_ms"],
                     "lat": lat[len(lat) // 2] if lat else None, "saltos": ESTADO["saltos"],
                     "fps": (len(ESTADO.get("fotos", [])) - 1) * 1000.0 /
                            max(1, ESTADO["fotos"][-1] - ESTADO["fotos"][0]) if len(ESTADO.get("fotos", [])) > 2 else None,
                     "n": ESTADO["n"], "robots": ESTADO["robots"],
                     "ahora": int(time.time() * 1000),
                     "ips": {k: v[0] + ":" + str(v[1]) for k, v in ESTADO["ips"].items()}}
            self._resp(json.dumps(d), "application/json")
        else:
            self._resp(HTML, "text/html; charset=utf-8")

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        try:
            o = json.loads(self.rfile.read(n))
            rid = str(o["id"])
            txt = o["txt"]
            with CANDADO:
                dest = ESTADO["ips"].get(rid)
            if o.get("ip"):
                dest = (o["ip"], 2027)
            if dest is None:
                self._resp(json.dumps({"ok": False, "error": "no sé la IP del robot %s" % rid}),
                           "application/json")
                return
            UDP.sendto(txt.encode(), dest)
            with CANDADO:
                registrar("orden", {"id": rid, "txt": txt})
            with CANDADO:
                ESTADO["ips"][rid] = dest
            self._resp(json.dumps({"ok": True, "a": "%s:%d" % dest}), "application/json")
        except Exception as e:  # noqa
            self._resp(json.dumps({"ok": False, "error": str(e)}), "application/json")


HTML = r"""<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Panel DynamiteCoders</title>
<style>
:root{--bg:#f4f5f7;--fg:#1d2330;--card:#fff;--line:#d5d9e0;--mute:#6b7280;--ok:#15803d;--bad:#b91c1c}
@media (prefers-color-scheme:dark){:root{--bg:#14171c;--fg:#e6e8eb;--card:#1d2128;--line:#343a44;--mute:#9aa3ad;--ok:#4ade80;--bad:#f87171}}
*{box-sizing:border-box}body{margin:0;font:14px system-ui,sans-serif;background:var(--bg);color:var(--fg)}
header{padding:10px 16px;display:flex;gap:16px;align-items:center;flex-wrap:wrap;border-bottom:1px solid var(--line)}
h1{font-size:16px;margin:0}.pill{padding:3px 10px;border-radius:99px;background:var(--card);border:1px solid var(--line)}
main{display:grid;grid-template-columns:minmax(300px,640px) 1fr;gap:16px;padding:16px}
@media (max-width:900px){main{grid-template-columns:1fr}}
canvas{width:100%;aspect-ratio:1;background:var(--card);border:1px solid var(--line);border-radius:8px;cursor:crosshair}
.card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:12px;margin-bottom:12px}
.card h2{font-size:14px;margin:0 0 8px}.mute{color:var(--mute)}.ok{color:var(--ok)}.bad{color:var(--bad)}
button{font:inherit;padding:5px 9px;border-radius:6px;border:1px solid var(--line);background:var(--bg);color:var(--fg);cursor:pointer;margin:2px}
button:hover{border-color:var(--fg)}input,select{font:inherit;padding:4px;border-radius:6px;border:1px solid var(--line);background:var(--bg);color:var(--fg)}
table{border-collapse:collapse;width:100%}td{padding:2px 6px;border-bottom:1px solid var(--line)}
#log{height:120px;overflow:auto;font:12px ui-monospace,monospace;white-space:pre-wrap}
</style></head><body>
<header><h1>Panel DynamiteCoders · Vision Rover Challenge</h1>
<span class="pill" id="con">sin conexión</span><span class="pill" id="fase">—</span>
<span class="pill" id="reloj">—</span><span class="pill" id="red">—</span></header>
<main><div><canvas id="c" width="900" height="900"></canvas>
<p class="mute">Clic en la cancha = "ir a ese punto" para el robot elegido (solo con MODO_PRUEBAS). Línea punteada = ruta; banda clara = camino reservado; flecha punteada = hacia dónde CREE el robot que miran sus paletas (tiene que coincidir con las paletas dibujadas).</p></div>
<div>
<div class="card"><h2>Robots</h2><div id="robots" class="mute">Esperando informes de los robots (DEPURACION_HOST en secretos.py)…</div></div>
<div class="card"><h2>Pruebas (MODO_PRUEBAS = True)</h2>
Robot <select id="rid"><option>10</option><option>11</option></select>
IP <input id="ip" size="13" placeholder="auto">
<div style="margin-top:6px">
<button onclick="o('PARAR')"><b>■ PARAR</b></button><button onclick="o('AUTO')">▶ Estrategia completa</button><br>
Girar a: <button onclick="o('GIRAR 0')">0° →</button><button onclick="o('GIRAR 90')">90° ↑</button><button onclick="o('GIRAR 180')">180° ←</button><button onclick="o('GIRAR 270')">270° ↓</button><br>
Agarrar: <button onclick="o('AGARRAR red')">rojo</button><button onclick="o('AGARRAR green')">verde</button><button onclick="o('AGARRAR blue')">azul</button>
Llevar: <button onclick="o('LLEVAR red')">rojo</button><button onclick="o('LLEVAR green')">verde</button><button onclick="o('LLEVAR blue')">azul</button><br>
Recto <input id="rs" size="3" value="1.5">s a <input id="rv" size="4" value="0.6"> <button onclick="recto()">Ir recto y medir</button>
<button onclick="agarre()">Medir AGARRE</button><br>
<button onclick="o('CALIBRAR')">🧭 Calibrar marcador</button> <span class="mute">(avanza recto ~5 celdas: dejalo en el medio mirando hacia adentro)</span></div>
<div id="medida" style="margin-top:6px"></div></div>
<div class="card"><h2>Telemetría</h2><table id="tab"></table></div>
<div class="card"><h2>Registro</h2><div id="log"></div></div>
</div></main>
<script>
const C=document.getElementById('c'),X=C.getContext('2d');let E=null,S=900,pad=40,esc=1;
const col={red:'#dc2626',green:'#16a34a',blue:'#2563eb'};
function log(t){const l=document.getElementById('log');l.textContent=new Date().toLocaleTimeString()+'  '+t+'\n'+l.textContent}
function P(c,r){return [pad+c*esc,pad+r*esc]}
function o(txt,extra){const id=document.getElementById('rid').value,ip=document.getElementById('ip').value.trim();
 fetch('/orden',{method:'POST',body:JSON.stringify({id,txt,ip:ip||undefined})}).then(r=>r.json()).then(j=>log('R'+id+' ← '+txt+(j.ok?'':'  ERROR: '+j.error)))}
function rob(id){return E&&E.tele?E.tele.rovers.find(r=>r.id==id):null}
function recto(){const id=+document.getElementById('rid').value,s=+document.getElementById('rs').value,v=+document.getElementById('rv').value;
 const a=rob(id);if(!a){log('no veo al robot '+id);return}o('RECTO '+s+' '+v);
 setTimeout(()=>{const b=rob(id);const d=Math.hypot(b.col-a.col,b.row-a.row);const dth=((b.theta-a.theta+540)%360)-180;
 document.getElementById('medida').innerHTML='Avanzó <b>'+d.toFixed(1)+'</b> celdas en '+s+' s → <b>'+(d/s).toFixed(1)+' celdas/s</b> a potencia '+v+
 ' (CELDAS_POR_S ≈ '+(d/s*0.6/v).toFixed(1)+' para VEL_CRUCERO 0.6). Giro final '+dth.toFixed(1)+'°';},(s+0.8)*1000)}
function agarre(){const id=+document.getElementById('rid').value,a=rob(id);if(!a)return;let best=null;
 for(const c of E.tele.cubes){const d=Math.hypot(c.col-a.col,c.row-a.row);if(!best||d<best.d)best={d,c}}
 document.getElementById('medida').innerHTML=best?('Cubo '+best.c.color+' a <b>'+best.d.toFixed(2)+'</b> celdas del robot. Si está metido entre las paletas tocando el frente, ese es el AGARRE (config.py).'):'sin cubos'}
C.addEventListener('click',ev=>{if(!E||!E.tele)return;const b=C.getBoundingClientRect();const x=(ev.clientX-b.left)*S/b.width,y=(ev.clientY-b.top)*S/b.height;
 const c=((x-pad)/esc).toFixed(1),r=((y-pad)/esc).toFixed(1);o('IR '+c+' '+r)});
function dib(){const t=E&&E.tele;X.clearRect(0,0,S,S);const fg=getComputedStyle(document.body).color;
 if(!t){X.fillStyle=fg;X.fillText('Sin telemetría',20,30);return}
 const g=t.grid;esc=(S-2*pad)/Math.max(g.cols,g.rows);
 X.strokeStyle='rgba(128,128,128,.15)';X.lineWidth=1;for(let i=0;i<=g.cols;i++){X.beginPath();X.moveTo(...P(i,0));X.lineTo(...P(i,g.rows));X.stroke()}
 for(let i=0;i<=g.rows;i++){X.beginPath();X.moveTo(...P(0,i));X.lineTo(...P(g.cols,i));X.stroke()}
 X.strokeStyle=fg;X.lineWidth=2;X.strokeRect(...P(0,0),g.cols*esc,g.rows*esc);
 const L=t.depot_size.length,F=t.depot_size.depth,m=t.cube_side*Math.SQRT2/2;
 for(const d of t.depots){const ab=Math.min(d.row,g.rows-d.row)<Math.min(d.col,g.cols-d.col);const w=ab?L:F,h=ab?F:L;
  X.fillStyle=col[d.color]+'33';X.fillRect(...P(d.col-w/2,d.row-h/2),w*esc,h*esc);
  X.setLineDash([4,4]);X.strokeStyle=col[d.color];X.lineWidth=1;X.strokeRect(...P(d.col-w/2+m,d.row-h/2+m),(w-2*m)*esc,(h-2*m)*esc);X.setLineDash([])}
 X.fillStyle=fg;X.beginPath();X.arc(...P(t.start.col,t.start.row),5,0,7);X.fill();
 for(const k in (E.robots||{})){const r=E.robots[k];if(E.ahora-r._ms>2000)continue;
  if(r.k&&r.k.length>1){X.strokeStyle='rgba(234,179,8,.25)';X.lineWidth=11*esc;X.lineCap='round';X.beginPath();X.moveTo(...P(...r.k[0]));for(const q of r.k)X.lineTo(...P(...q));X.stroke()}
  if(r.r&&r.pos){X.strokeStyle=k=='10'?'#9333ea':'#0891b2';X.lineWidth=2;X.setLineDash([6,5]);X.beginPath();X.moveTo(...P(...r.pos));for(const q of r.r)X.lineTo(...P(...q));X.stroke();X.setLineDash([])}}
 for(const c of t.cubes){const s=t.cube_side;X.globalAlpha=c.age_ms>1000?0.35:1;X.fillStyle=col[c.color]||'#888';X.fillRect(...P(c.col-s/2,c.row-s/2),s*esc,s*esc);
  if(c.in_depot){X.strokeStyle=fg;X.lineWidth=3;X.strokeRect(...P(c.col-s/2,c.row-s/2),s*esc,s*esc)}X.globalAlpha=1}
 for(const r of t.rovers){const Rr=(E.robots||{})[r.id],dm=(Rr&&Rr.dm)?Rr.dm:0,th=(r.theta+dm)*Math.PI/180,cx=Math.cos(th),cy=-Math.sin(th);
  const pt=(a,i)=>P(r.col+a*cx+i*cy,r.row+a*cy-i*cx);X.globalAlpha=r.age_ms>500?0.4:1;
  X.strokeStyle=r.id==10?'#9333ea':'#0891b2';X.lineWidth=3;X.beginPath();
  for(const [a,i] of [[-2,-2.6],[3.5,-2.6],[3.5,2.6],[-2,2.6],[-2,-2.6]])X.lineTo(...pt(a,i));X.stroke();
  for(const s of [-1,1]){X.beginPath();X.moveTo(...pt(3.5,2.6*s));X.lineTo(...pt(7,2.6*s));X.stroke()}
  X.fillStyle=X.strokeStyle;X.font='bold 15px system-ui';X.fillText(r.id,...P(r.col-1,r.row+0.6));X.globalAlpha=1;
  const R=(E.robots||{})[r.id];if(R&&R.h!=null&&E.ahora-R._ms<2000){const h=R.h*Math.PI/180;X.setLineDash([3,3]);X.lineWidth=2;
   X.beginPath();X.moveTo(...P(r.col,r.row));X.lineTo(...P(r.col+9*Math.cos(h),r.row-9*Math.sin(h)));X.stroke();X.setLineDash([])}}}
function tabla(){const t=E.tele;if(!t)return;const filas=[['versión',t.v+(t.v!==3?'  ⚠ el código espera v3':'')],['seq',t.seq+'  (saltos: '+E.saltos+')'],
 ['cámara',(E.fps==null?'—':E.fps.toFixed(1)+' fotos nuevas/s')+(E.fps!=null&&E.fps<8?'  ⚠ lenta: el robot lo compensa, pero va más despacio':'')],['atraso de cada foto',E.lat==null?'—':E.lat+' ms'],['cancha',t.grid.cols+'×'+t.grid.rows+' celdas de '+t.grid.cell_mm+' mm']];
 for(const r of t.rovers)filas.push(['rover '+r.id,'('+r.col.toFixed(2)+', '+r.row.toFixed(2)+')  θ '+r.theta.toFixed(1)+'°  edad '+r.age_ms+' ms']);
 for(const c of t.cubes)filas.push(['cubo '+c.color,'('+c.col.toFixed(2)+', '+c.row.toFixed(2)+')  edad '+c.age_ms+' ms'+(c.in_depot?'  ✔ ENTREGADO':'')]);
 document.getElementById('tab').innerHTML=filas.map(f=>'<tr><td class="mute">'+f[0]+'</td><td>'+f[1]+'</td></tr>').join('')}
function robots(){const R=E.robots||{};const ks=Object.keys(R);if(!ks.length)return;
 document.getElementById('robots').innerHTML=ks.sort().map(k=>{const r=R[k],viejo=E.ahora-r._ms>2000;
 return '<div style="margin-bottom:6px"><b>Rover '+k+'</b> '+(viejo?'<span class="bad">sin informes</span>':'')+' — <b>'+r.e+'</b>'+(r.t?' · cubo <b style="color:'+col[r.t]+'">'+r.t+'</b>':'')+
 (r.f?' · <span class="bad">frena: '+r.f+'</span>':'')+'<br><span class="mute">'+(r.m||'')+' · entregó '+r.n+' · giroscopio '+(r.giro?(r.inv?'<span class="bad">INVERTIDO (corregir GIRO_SIGNO)</span>':'ok'):'no')+
 ' · radio '+(r.par?'<span class="ok">ok</span>':'<span class="bad">sin datos del otro</span>')+' · IP '+(E.ips[k]||'?')+
 '<br>marcador: '+(r.dm==null?'?':(r.dm==0?'alineado (0°)':'<b>girado '+r.dm+'°</b> (corregido)'))+' · tramos medidos '+(r.dn||0)+(r.de!=null?' (último error '+r.de+'°)':'')+
 ' · '+(r.hz||'?')+' vueltas/s · atraso medido '+(r.lat||'?')+' ms · escala '+(r.esc||'?')+(r.hz&&r.hz<12?' <span class="bad">(lento)</span>':'')+
 '<br>WiFi: señal '+(r.rssi==null?'?':r.rssi+' dBm'+(r.rssi<-75?' <span class="bad">(débil)</span>':''))+' · cortes con la visión '+(r.cortes||0)+(r.err?' (último: '+r.err+')':'')+
 ' · memoria libre '+(r.mem==null?'?':r.mem)+(r.rr&&r.rr!='POWER_ON'&&r.rr!='?'?' · <span class="bad">se reinició solo ('+r.rr+')</span>':'')+
 (r.lg?'<br>'+r.lg.map(x=>x[1]).join('<br>'):'')+'</span></div>'}).join('')}
async function bucle(){try{E=await (await fetch('/estado')).json();
 document.getElementById('con').innerHTML=E.conectado?'<span class="ok">● visión conectada</span>':'<span class="bad">● sin visión</span>';
 const t=E.tele;if(t){document.getElementById('fase').textContent='Fase: '+t.phase;const ms=t.phase=='READY'?t.clock.remaining_ms:t.clock.elapsed_ms;
 document.getElementById('reloj').textContent=(t.phase=='READY'?'faltan ':'')+Math.floor(ms/60000)+':'+String(Math.floor(ms/1000)%60).padStart(2,'0');
 const ent=t.cubes.filter(c=>c.in_depot).length;document.getElementById('red').textContent='Entregados: '+ent+'/'+t.cubes.length}
 dib();tabla();robots()}catch(e){}setTimeout(bucle,100)}
bucle();
</script></body></html>"""


def hilo_baliza():
    """Cada segundo avisa por difusión "acá está la compu de la visión". Los
    robots (en modo pruebas) usan la IP de quien manda esto si la de
    secretos.py no anda (por ejemplo, conectados al WiFi del celular)."""
    destinos = {"255.255.255.255"}
    try:
        for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
            if not ip.startswith("127."):
                destinos.add(ip.rsplit(".", 1)[0] + ".255")
    except OSError:
        pass
    while True:
        for d in list(destinos):
            try:
                UDP.sendto(b"BALIZA", (d, 2027))
            except OSError:
                pass
        time.sleep(1.0)


def main():
    global UDP
    ap = argparse.ArgumentParser()
    ap.add_argument("--vision", default="127.0.0.1", help="IP de la compu con la visión o el simulador")
    ap.add_argument("--puerto", type=int, default=2026)
    ap.add_argument("--web", type=int, default=8080)
    ap.add_argument("--no-abrir", action="store_true")
    a = ap.parse_args()
    carpeta = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "registros"))
    try:
        os.makedirs(carpeta, exist_ok=True)
        nombre = os.path.join(carpeta, time.strftime("sesion_%Y%m%d_%H%M%S.ndjson"))
        REG["f"] = open(nombre, "w", encoding="utf-8")
        print("Registro de la sesión:", nombre)
    except OSError as e:
        print("No pude crear el registro:", e)
    UDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UDP.bind(("0.0.0.0", 2028))
    try:
        UDP.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except OSError:
        pass
    threading.Thread(target=hilo_vision, args=(a.vision, a.puerto), daemon=True).start()
    threading.Thread(target=hilo_udp, daemon=True).start()
    threading.Thread(target=hilo_baliza, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", a.web), Pagina)
    url = "http://localhost:%d" % a.web
    print("Panel en", url, "  (visión:", a.vision, a.puerto, ")  Ctrl+C para salir")
    print("Si Windows pregunta por el firewall, marcá redes PRIVADAS y PÚBLICAS (los robots mandan informes al puerto 2028).")
    if not a.no_abrir:
        try:
            webbrowser.open(url)
        except Exception:  # noqa
            pass
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
