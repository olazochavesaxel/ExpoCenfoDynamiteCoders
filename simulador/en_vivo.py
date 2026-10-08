# en_vivo.py — simulador EN TIEMPO REAL con los dos rovers virtuales corriendo
# la lógica real (firmware/rover). Publica la telemetría igual que el sistema
# oficial (TCP 2026, contrato v3), así que el panel —o cualquier cliente— se
# conecta como si fuera la cancha de verdad.
#
#   python simulador/en_vivo.py
#   python herramientas/panel.py          (en otra terminal) y abrir el navegador
#
# Comandos en esta terminal: ready | stop | abort | nuevo | quit
#   nuevo = otra ubicación al azar para los cubos (vuelve a IDLE)

import argparse
import os
import random
import socket
import sys
import threading
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, "..", "firmware", "rover"))
sys.path.insert(0, AQUI)

import cancha as C          # noqa: E402
import cerebro              # noqa: E402
import mundo                # noqa: E402
from correr import HwSim, EnlaceSim, config_para   # noqa: E402


class Servidor:
    """TCP NDJSON, 'el último valor gana', como el sistema oficial."""

    def __init__(self, puerto):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(("0.0.0.0", puerto))
        self.s.listen(8)
        self.s.setblocking(False)
        self.clientes = []

    def publicar(self, linea):
        try:
            while True:
                c, dir_ = self.s.accept()
                c.setblocking(False)
                self.clientes.append(c)
                print("[cliente] conectado", dir_)
        except BlockingIOError:
            pass
        dato = (linea + "\n").encode()
        for c in list(self.clientes):
            try:
                c.send(dato)
            except BlockingIOError:
                pass                       # cliente lento: se pisa
            except OSError:
                self.clientes.remove(c)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto", type=int, default=2026)
    ap.add_argument("--prep", type=float, default=5.0, help="segundos de READY (la oficial usa 60)")
    ap.add_argument("--semilla", type=int, default=None)
    ap.add_argument("--panel", default="127.0.0.1", help="IP del panel para los informes")
    ap.add_argument("--marcador", default=None,
                    help="simular marcadores pegados girados, grados para el 10 y el 11, ej: 90,-90")
    a = ap.parse_args()

    estado = {"k": None, "cer": None, "socks": None, "salir": False}
    rng = random.Random(a.semilla)

    def armar():
        cubos = C.cubos_al_azar(rng)
        marc = None
        if a.marcador:
            g = [float(x) for x in a.marcador.split(",")]
            marc = {10: g[0], 11: g[-1]}
        k = C.Cancha(cubos, [(10, 4.0, 17.5, 0.0), (11, 4.0, 25.5, 0.0)], semilla=rng.randint(0, 10**6),
                     marcador_girado=marc)
        k.prep = a.prep
        k.ts0 = time.time() * 1000 - k.latencia * 1000
        bus = {10: [], 11: []}
        cer = {}
        socks = {}
        for rid in (10, 11):
            cfg = config_para(rid, MODO_PRUEBAS=True)
            cer[rid] = cerebro.Cerebro(cfg, HwSim(k.rovers[rid]), EnlaceSim(bus, rid, 0.05, rng),
                                       lambda t, rid=rid: print("R%d %s" % (rid, t)))
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("127.0.0.1", 0))
            s.setblocking(False)
            socks[rid] = s
        if estado["socks"]:
            for s in estado["socks"].values():
                s.close()
        estado.update(k=k, cer=cer, socks=socks)
        print("Cubos:", {c: (round(p[0], 1), round(p[1], 1)) for c, p in cubos.items()})

    armar()
    srv = Servidor(a.puerto)

    def teclado():
        for linea in sys.stdin:
            cmd = linea.strip().lower()
            k = estado["k"]
            if cmd == "ready" and k.fase in ("IDLE", "FINISHED"):
                k.preparar()
            elif cmd == "stop" and k.fase == "RUNNING":
                k._fin("detenida_por_operador")
            elif cmd == "abort":
                k.fase = "IDLE"
            elif cmd == "nuevo":
                armar()
            elif cmd == "quit":
                estado["salir"] = True
                return
            print("[fase]", estado["k"].fase)

    threading.Thread(target=teclado, daemon=True).start()
    print("Simulador en vivo publicando en el puerto %d. Escribí 'ready' para arrancar." % a.puerto)

    t0 = time.monotonic()
    prox_cam = prox_rov = prox_inf = 0.0
    pendiente = []
    ultima = {10: None, 11: None}
    fase_ant = None
    while not estado["salir"]:
        ahora = time.monotonic() - t0
        k, cer, socks = estado["k"], estado["cer"], estado["socks"]
        if ahora >= prox_cam:
            prox_cam = ahora + 0.05
            linea = k.foto()
            srv.publicar(linea)
            pendiente.append((ahora + k.latencia, linea))
        while pendiente and pendiente[0][0] <= ahora:
            l = pendiente.pop(0)[1]
            ultima = {10: l, 11: l}
        if ahora >= prox_rov:
            prox_rov = ahora + 0.033
            ms = int(ahora * 1000)
            for rid, c in cer.items():
                m = mundo.interpretar(ultima[rid], ms) if ultima[rid] else None
                ultima[rid] = None
                orden = None
                try:
                    data, _ = socks[rid].recvfrom(256)
                    orden = data.decode()
                except BlockingIOError:
                    pass
                c.paso(ms, m, orden)
        if ahora >= prox_inf:
            prox_inf = ahora + 0.2
            for rid, c in cer.items():
                try:
                    socks[rid].sendto(c.informe_texto().encode(), (a.panel, 2028))
                except OSError:
                    pass
        k.paso(0.005)
        if k.fase != fase_ant:
            print("[fase]", k.fase, k.fin_motivo or "")
            fase_ant = k.fase
            if k.fase == "FINISHED":
                print("Resultado:", {c: round(t, 1) for c, (t, _) in k.tiempo_entrada.items()},
                      "-> escribí 'nuevo' y 'ready' para otra ronda")
        dormir = 0.005 - ((time.monotonic() - t0) - ahora)
        if dormir > 0:
            time.sleep(dormir)


if __name__ == "__main__":
    main()
