# correr.py — corre la lógica REAL de los rovers (firmware/rover/cerebro.py)
# contra la cancha simulada, sin robots ni cámara.
#
#   python simulador/correr.py                 # 20 escenarios al azar, resumen
#   python simulador/correr.py -n 100          # 100 escenarios
#   python simulador/correr.py --ver 7         # escenario 7 con detalle + imagen
#   python simulador/correr.py --sin-radio     # como si ESP-NOW no anduviera
#
# Para ver la simulación EN VIVO en el panel, usar simulador/en_vivo.py.

import argparse
import json
import math
import os
import random
import sys
import types

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, "..", "firmware", "rover"))
sys.path.insert(0, AQUI)

import cancha as C          # noqa: E402
import cerebro              # noqa: E402
import mundo                # noqa: E402

PERIODO_CAMARA = 0.05       # 20 Hz como el sistema oficial
PERIODO_ROVER = float(os.environ.get("PERIODO_ROVER", "0.033"))   # ~30 vueltas/s (ESP32: menos)
DT = 0.005


def config_para(rid, **cambios):
    import config as base
    c = types.SimpleNamespace()
    for k in dir(base):
        if k.isupper():
            setattr(c, k, getattr(base, k))
    c.ROBOT_ID = rid
    for k, v in cambios.items():
        setattr(c, k, v)
    return c


class HwSim:
    def __init__(self, r, con_giro=True):
        self.r = r
        self.con_giro = con_giro
        self.color = (0, 0, 0)

    def motores(self, izq, der):
        self.r.ul, self.r.ur = izq, der

    def giro_dps(self):
        return self.r.giro_dps() if self.con_giro else None

    def led(self, rgb):
        self.color = rgb


class EnlaceSim:
    """Radio entre los dos rovers, con pérdidas."""
    def __init__(self, bus, rid, perdida, rng):
        self.bus, self.id, self.perdida, self.rng = bus, rid, perdida, rng

    def enviar(self, txt):
        for k in self.bus:
            if k != self.id and self.rng.random() >= self.perdida:
                self.bus[k].append(txt)

    def recibir(self):
        q = self.bus[self.id]
        out = list(q)
        q.clear()
        return out


class Partida:
    def __init__(self, semilla, cubos=None, sin_radio=False, sin_giro=False,
                 signo_giro=(1.0, 1.0), silencio=True, cfg_extra=None, solo=False,
                 vmax=None, latencia_ms=70, marcador=None, fps=None, real=False, peor=False, oficial=False):
        rng = random.Random(semilla)
        if cubos is None:
            cubos = C.cubos_oficiales(rng) if oficial else C.cubos_al_azar(rng)
        self.cubos_ini = dict(cubos)
        orden = [(10, 4.0, 17.5, 0.0), (11, 4.0, 25.5, 0.0)]
        if rng.random() < 0.5:
            orden = [(10, 4.0, 25.5, 0.0), (11, 4.0, 17.5, 0.0)]
        if solo:
            orden = orden[:1]
        self.k = C.Cancha(cubos, orden, semilla=semilla, signo_giro=signo_giro, latencia_ms=latencia_ms,
                          marcador_girado=marcador)
        if vmax:
            for r in self.k.rovers.values():
                r.vmax = vmax
        self.cortes = {}
        if real:
            # motores flojos (batería a medio cargar): arrancan con más potencia
            for r in self.k.rovers.values():
                r.zm = rng.uniform(0.19, 0.25)
            # cortes de WiFi de 2 a 6 s cada tanto, como en casa
            for o in orden:
                t, lista = 20.0, []
                while t < 620:
                    t += rng.uniform(40, 120)
                    lista.append((t, t + rng.uniform(2.0, 6.0)))
                self.cortes[o[0]] = lista
            self.k.borroso = True
            self.k.ver_cubo = {"red": 0.9, "green": 0.6, "blue": 0.45}
        if peor:
            # la prueba del 5-oct: baterías cargadas (motores MÁS rápidos que lo
            # que el robot calcula), 1,6 fotos/s con 0,65 s de atraso, el
            # marcador casi nunca se ve andando y el rojo se ve poco
            for r in self.k.rovers.values():
                r.zm = rng.uniform(0.08, 0.14)
            self.k.borroso_fuerte = True
            self.k.ver_cubo = {"red": 0.4, "green": 0.95, "blue": 0.55}
        self.k.preparar()
        bus = {10: [], 11: []}
        perdida = 1.0 if sin_radio else 0.05
        self.logs = []
        self.cer = {}
        for rid in [o[0] for o in orden]:
            def log(t, rid=rid):
                self.logs.append("R%d %s" % (rid, t))
                if not silencio:
                    print("R%d %s" % (rid, t))
            if cfg_extra is None and os.environ.get("SIM_CFG"):
                cfg_extra = json.loads(os.environ["SIM_CFG"])   # para probar variantes de config.py
            cfg = config_para(rid, **(cfg_extra or {}))
            self.cer[rid] = cerebro.Cerebro(cfg, HwSim(self.k.rovers[rid], not sin_giro),
                                            EnlaceSim(bus, rid, perdida, rng), log)
            self.cer[rid].relanzar_errores = True   # en el simulador queremos ver los errores
        self.prox_cam = 0.0
        self.prox_pub = 0.0
        self.publicada = None
        self.periodo_foto = 1.0 / fps if fps else PERIODO_CAMARA
        self.prox_rov = {o[0]: 0.007 * n for n, o in enumerate(orden)}
        self.recibido = {o[0]: None for o in orden}
        self.traza = []

    def correr(self, t_max=620.0):
        k = self.k
        while k.t < t_max:
            # la cámara saca una foto cada periodo_foto; la visión la publica
            # (repetida) 20 veces por segundo hasta que llega la siguiente
            if k.t >= self.prox_cam:
                self.prox_cam += self.periodo_foto
                k.cola.append((k.t + k.latencia, k.foto()))
            while k.cola and k.cola[0][0] <= k.t:
                self.publicada = k.cola.pop(0)[1]
            if k.t >= self.prox_pub:
                self.prox_pub += PERIODO_CAMARA
                if self.publicada is not None:
                    for rid in self.recibido:
                        self.recibido[rid] = self.publicada
            for rid, c in self.cer.items():
                if k.t >= self.prox_rov[rid]:
                    self.prox_rov[rid] += PERIODO_ROVER
                    ahora = int(k.t * 1000)
                    m = None
                    if any(a <= k.t < b for a, b in self.cortes.get(rid, ())):
                        self.recibido[rid] = None        # corte de WiFi: no llega nada
                    if self.recibido[rid] is not None:
                        m = mundo.interpretar(self.recibido[rid], ahora)
                        self.recibido[rid] = None
                    c.paso(ahora, m)
            k.paso(DT)
            if int(k.t / 0.25) != int((k.t - DT) / 0.25):
                self.traza.append((k.t, {r: (v.x, v.y, v.th) for r, v in k.rovers.items()},
                                   {c: tuple(p) for c, p in k.cubos.items()}))
            if k.fase == "FINISHED":
                break
        return self.resultado()

    def resultado(self):
        k = self.k
        entregados = [c for c in k.cubos if k.in_depot.get(c)]
        por_rover = {}
        for c in entregados:
            quien = k.tiempo_entrada.get(c, (0, None))[1]
            por_rover[quien] = por_rover.get(quien, 0) + 1
        tiempos = sorted(t for c, (t, _) in k.tiempo_entrada.items() if k.in_depot.get(c))
        valido = len(entregados) < 3 or all(por_rover.get(r, 0) >= 1 for r in (10, 11))
        return {
            "cubos": len(entregados), "tiempo": tiempos[-1] if tiempos else None,
            "tiempos": tiempos, "por_rover": por_rover, "valido_12_2_13": valido,
            "choques": k.choques, "caidos": len(k.caidos),
            "eventos": list(k.log),
        }


def dibujar(partida, archivo):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-3.5, 46.5)
    ax.set_ylim(46.5, -3.5)
    ax.set_aspect("equal")
    ax.add_patch(Rectangle((0, 0), 43, 43, fill=False, lw=1.5))
    colores = {"red": "#d62728", "green": "#2ca02c", "blue": "#1f77b4"}
    for c, z in C.ZONAS.items():
        if abs(z[1] - 3.75) < 1 or abs(z[1] - 39.25) < 1:
            w, h = 10, 7.5
        else:
            w, h = 7.5, 10
        ax.add_patch(Rectangle((z[0] - w / 2, z[1] - h / 2), w, h, color=colores[c], alpha=0.18))
    for c, p in partida.cubos_ini.items():
        ax.plot(p[0], p[1], "s", mfc="none", mec=colores[c], ms=10)
    for rid, est in (("10", "-"), ("11", "--")):
        if int(rid) not in partida.traza[0][1]:
            continue
        xs = [t[1][int(rid)][0] for t in partida.traza]
        ys = [t[1][int(rid)][1] for t in partida.traza]
        ax.plot(xs, ys, est, color="k", lw=1, label="rover " + rid)
        ax.plot(xs[-1], ys[-1], "o", color="k")
    for c in colores:
        xs = [t[2][c][0] for t in partida.traza if c in t[2]]
        ys = [t[2][c][1] for t in partida.traza if c in t[2]]
        ax.plot(xs, ys, ":", color=colores[c], lw=1.5)
        ax.plot(xs[-1], ys[-1], "s", color=colores[c], ms=10)
    r = partida.resultado()
    ax.set_title("cubos %d  tiempo %s  choques %d" % (r["cubos"], r["tiempo"] and "%.1f s" % r["tiempo"], r["choques"]))
    ax.legend(loc="lower right")
    fig.savefig(archivo, dpi=90, bbox_inches="tight")
    plt.close(fig)


def _uno(s, sin_radio, sin_giro, sg, vmax=None, lat=70, marcador=None, fps=None, real=False, peor=False,
        oficial=False):
    p = Partida(s, sin_radio=sin_radio, sin_giro=sin_giro, signo_giro=sg, vmax=vmax, latencia_ms=lat,
                marcador=marcador, fps=fps, real=real, peor=peor, oficial=oficial)
    r = p.correr()
    r.pop("eventos", None)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, default=20)
    ap.add_argument("--desde", type=int, default=0, help="primer escenario")
    ap.add_argument("--ver", type=int, default=None)
    ap.add_argument("--sin-radio", action="store_true")
    ap.add_argument("--sin-giro", action="store_true")
    ap.add_argument("--giro-invertido", action="store_true")
    ap.add_argument("--imagen", default="simulacion.png")
    ap.add_argument("--vmax", type=float, default=None, help="velocidad máxima de rueda (celdas/s)")
    ap.add_argument("--latencia", type=int, default=70, help="retraso de la cámara (ms)")
    ap.add_argument("--fps", type=float, default=None,
                    help="fotos por segundo de la cámara (la de casa: 2.6 con --latencia 350)")
    ap.add_argument("--casa", action="store_true",
                    help="como la cancha de casa: cámara lenta (2,6 fotos/s, 350 ms) y robots rápidos")
    ap.add_argument("--real", action="store_true",
                    help="como la última prueba en casa: 1,7 fotos/s, 450 ms, robots rápidos, "
                         "marcadores borrosos al moverse y cubos verde/azul que se pierden")
    ap.add_argument("--oficial", action="store_true",
                    help="cubos como el generador oficial de la organización (dificultad al azar)")
    ap.add_argument("--peor", action="store_true",
                    help="como la prueba del 5-oct: 1,6 fotos/s, 650 ms, marcador casi invisible andando")
    ap.add_argument("--marcador", default=None,
                    help="marcadores pegados girados, grados para el 10 y el 11, ej: 90,-90")
    a = ap.parse_args()
    sg = (-1.0, -1.0) if a.giro_invertido else (1.0, 1.0)
    if a.peor:
        a.real = True
        a.fps = a.fps or 1.6
        a.latencia = a.latencia if a.latencia != 70 else 650
    if a.real:
        a.casa = True
        a.fps = a.fps or 1.7
        if a.latencia == 70:
            a.latencia = 450
        a.vmax = a.vmax or 38.0
    if a.casa:
        a.fps = a.fps or 2.6
        if a.latencia == 70:
            a.latencia = 350
        a.vmax = a.vmax or 27.0
    marc = None
    if a.marcador:
        g = [float(x) for x in a.marcador.split(",")]
        marc = {10: g[0], 11: g[-1]}
    if a.ver is not None:
        p = Partida(a.ver, sin_radio=a.sin_radio, sin_giro=a.sin_giro, signo_giro=sg, silencio=False,
                    marcador=marc, fps=a.fps, latencia_ms=a.latencia, vmax=a.vmax, real=a.real)
        r = p.correr()
        for e in r["eventos"]:
            print("  *", e)
        print(r)
        dibujar(p, a.imagen)
        print("Imagen:", a.imagen)
        return
    import multiprocessing as mp
    with mp.Pool(max(1, (os.cpu_count() or 2))) as pool:
        tot = pool.starmap(_uno, [(s, a.sin_radio, a.sin_giro, sg, a.vmax, a.latencia, marc, a.fps, a.real, a.peor, a.oficial) for s in range(a.desde, a.desde + a.n)])
    for s, r in enumerate(tot, a.desde):
        print("escenario %3d: cubos=%d tiempo=%s por_rover=%s choques=%d caidos=%d %s" % (
            s, r["cubos"], r["tiempo"] and "%.1f" % r["tiempo"], r["por_rover"], r["choques"],
            r["caidos"], "" if r["valido_12_2_13"] else "<-- NO cumple 12.2.13"))
    tres = [r for r in tot if r["cubos"] == 3 and r["valido_12_2_13"]]
    print()
    print("Completos y válidos: %d/%d" % (len(tres), len(tot)))
    if tres:
        ts = sorted(r["tiempo"] for r in tres)
        print("Tiempo del 3er cubo: mediana %.1f s, peor %.1f s" % (ts[len(ts) // 2], ts[-1]))
    print("Choques totales:", sum(r["choques"] for r in tot), " cubos caídos:", sum(r["caidos"] for r in tot))


if __name__ == "__main__":
    main()
