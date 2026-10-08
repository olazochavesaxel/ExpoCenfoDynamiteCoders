# cancha.py — física simplificada de la cancha del Vision Rover Challenge,
# para probar la lógica de los rovers SIN robots ni cámara.
#
# Simula:
#   - dos CenfoBots de tracción diferencial (con zona muerta del motor,
#     inercia, motores desparejos y giroscopio con ruido y sesgo)
#   - las paletas delanteras: el cubo queda "enjaulado" entre ellas
#   - cubos que se empujan, chocan entre sí y pueden caerse del tablero
#   - la cámara: ruido, retraso, oclusión y pérdidas, en formato CONTRATO v3
#   - el árbitro: in_depot con 1 s de permanencia y 2,5 mm de tolerancia
#
# Las medidas están en celdas (20 mm). Es un modelo: sirve para encontrar
# errores de lógica, no reemplaza probar en la cancha real.

import json
import math
import random

GRID = (43, 43)
ZONAS = {"green": (21.5, 3.75), "red": (39.25, 21.5), "blue": (21.5, 39.25)}
ZONA_TAM = (10.0, 7.5)
LADO_CUBO = 3.0
SALIDA = (3.75, 21.5)
MARGEN_FISICO = 3.5          # el tablero físico sobresale 3,5 celdas de la cancha
R_CUBO = 1.55                # el cubo se modela como un círculo
TOLERANCIA = 2.5 / 20.0      # holgura del árbitro (celdas)
RAD = math.pi / 180.0

# Forma del robot en su propio marco (adelante, izquierda), en celdas:
CUERPO = (-2.0, 3.5, -2.6, 2.6)
PALETA_IZQ = (3.5, 7.0, 2.35, 2.85)
PALETA_DER = (3.5, 7.0, -2.85, -2.35)
# Las paletas del CenfoBot se abren hacia afuera (ver Cenfobot_Rover.jpeg): la
# punta queda más separada que la base y el cubo entra "embudado". Se arma con
# dos tramos por paleta.
PIEZAS = (CUERPO,
          (3.5, 5.25, 2.35, 2.85), (5.25, 7.0, 2.85, 3.35),
          (3.5, 5.25, -2.85, -2.35), (5.25, 7.0, -3.35, -2.85))


def vec(th):
    return math.cos(th * RAD), -math.sin(th * RAD)


def falta_zona(p, z):
    if abs(z[1] - 0) < 5 or abs(z[1] - GRID[1]) < 5:
        sc, sr = ZONA_TAM[0] / 2, ZONA_TAM[1] / 2
    else:
        sc, sr = ZONA_TAM[1] / 2, ZONA_TAM[0] / 2
    m = LADO_CUBO * math.sqrt(2) / 2
    ec = max(0.0, abs(p[0] - z[0]) - (sc - m))
    er = max(0.0, abs(p[1] - z[1]) - (sr - m))
    return math.hypot(ec, er)


class RoverSim:
    def __init__(self, rid, x, y, th, rng, vmax=14.0, zona_muerta=0.17,
                 ganancia_der=None, signo_giro=1.0):
        self.id = rid
        self.x, self.y, self.th = x, y, th
        self.vl = self.vr = 0.0
        self.ul = self.ur = 0.0
        self.vmax = vmax
        self.zm = zona_muerta
        self.gder = ganancia_der if ganancia_der is not None else rng.uniform(0.92, 1.06)
        self.gizq = 1.0
        self.base = 5.0
        self.omega = 0.0           # grados/s reales
        self.sesgo = rng.uniform(-1.0, 1.0)
        self.signo_giro = signo_giro
        self.rng = rng
        self.caido = False
        self.carga = 1.0           # <1 mientras empuja un cubo (cuesta más avanzar)

    def _vel(self, u, g):
        if abs(u) < self.zm:
            return 0.0
        s = 1 if u > 0 else -1
        return s * g * self.carga * self.vmax * (abs(u) - self.zm) / (1 - self.zm)

    def paso(self, dt):
        tau = 0.12
        a = min(1.0, dt / tau)
        self.vl += (self._vel(self.ul, self.gizq) - self.vl) * a
        self.vr += (self._vel(self.ur, self.gder) - self.vr) * a
        v = (self.vl + self.vr) / 2
        self.omega = (self.vr - self.vl) / self.base / RAD
        self.th = (self.th + self.omega * dt) % 360
        c, s = vec(self.th)
        self.x += v * c * dt
        self.y += v * s * dt

    def giro_dps(self):
        return self.signo_giro * (self.omega + self.sesgo + self.rng.gauss(0, 0.8))

    def a_mundo(self, a, i):
        c, s = vec(self.th)
        # izquierda = (s, -c)
        return self.x + a * c + i * s, self.y + a * s - i * c

    def a_local(self, p):
        c, s = vec(self.th)
        dx, dy = p[0] - self.x, p[1] - self.y
        return dx * c + dy * s, dx * s - dy * c


class Cancha:
    def __init__(self, cubos, rovers, semilla=1, ruido=True, latencia_ms=70,
                 perdida_rover=0.02, signo_giro=(1.0, 1.0), mover_cubos_al_tocar=True,
                 marcador_girado=None):
        self.rng = random.Random(semilla)
        self.t = 0.0
        self.cubos = {k: list(v) for k, v in cubos.items()}
        self.caidos = set()
        self.rovers = {}
        for n, (rid, x, y, th) in enumerate(rovers):
            self.rovers[rid] = RoverSim(rid, x, y, th, self.rng, signo_giro=signo_giro[n])
        self.ruido = ruido
        # grados que el marcador está girado respecto de las paletas, por rover
        # (la cámara publica theta = rumbo real + esto)
        self.marcador_girado = marcador_girado or {}
        self.borroso = False           # cámara de casa: marcadores borrosos y cubos que se pierden
        self.ver_cubo = {}             # color -> probabilidad de verlo en cada foto
        self.latencia = latencia_ms / 1000.0
        self.perdida = perdida_rover
        self.seq = 0
        self.ultimo_visto = {}
        self.cola = []            # (t_entrega, linea)
        self.fase = "IDLE"
        self.t_fase = 0.0
        self.prep = 3.0
        self.duracion = 600.0
        self.dentro_desde = {}
        self.in_depot = {}
        self.tiempo_entrada = {}
        self.ultimo_toque = {}
        self.choques = 0
        self._en_choque = False
        self.fin_motivo = None
        self.t_run = 0.0
        self.fin_t = None
        self.log = []
        self.ts0 = 1785000000000      # en vivo se reemplaza por la hora real

    # --------------------------------------------------------------- física
    def paso(self, dt):
        self.t += dt
        if self.fase == "READY" and self.t - self.t_fase >= self.prep:
            self.fase, self.t_fase = "RUNNING", self.t
            self.t_run = self.t
        if self.fase == "RUNNING" and self.t - self.t_fase >= self.duracion:
            self._fin("tiempo_agotado")
        viejos = {k: (r.x, r.y, r.th) for k, r in self.rovers.items()}
        for r in self.rovers.values():
            if not r.caido:
                r.paso(dt)
        self._choque_rovers(viejos)
        for r in self.rovers.values():      # empujar un cubo frena (motores flojos)
            r.carga = 0.6 if r.id in getattr(self, "_tocando", ()) else 1.0
        self._tocando = set()
        for _ in range(3):
            for r in self.rovers.values():
                for color in self.cubos:
                    if color not in self.caidos:
                        self._empujar(r, color)
            self._cubos_entre_si()
        for color, p in self.cubos.items():
            if color in self.caidos:
                continue
            if not (-MARGEN_FISICO < p[0] < GRID[0] + MARGEN_FISICO and
                    -MARGEN_FISICO < p[1] < GRID[1] + MARGEN_FISICO):
                self.caidos.add(color)
                self.log.append("%.1f s: cubo %s se CAYO del tablero" % (self.t, color))
        for r in self.rovers.values():
            if not r.caido and not (-MARGEN_FISICO < r.x < GRID[0] + MARGEN_FISICO and
                                    -MARGEN_FISICO < r.y < GRID[1] + MARGEN_FISICO):
                r.caido = True
                self.log.append("%.1f s: rover %d se CAYO del tablero" % (self.t, r.id))
        self._arbitro()

    def _choque_rovers(self, viejos):
        ks = list(self.rovers)
        if len(ks) < 2:
            return
        a, b = self.rovers[ks[0]], self.rovers[ks[1]]
        choca = self._se_tocan(a, b)
        if choca:
            for k in ks:
                r = self.rovers[k]
                r.x, r.y, r.th = viejos[k]
                r.vl = r.vr = 0.0
            if not self._en_choque:
                self.choques += 1
                self.log.append("%.1f s: CHOQUE entre rovers" % self.t)
        self._en_choque = choca

    def _se_tocan(self, a, b):
        # muestreamos puntos del contorno de cada robot contra el rectángulo del otro
        for r1, r2 in ((a, b), (b, a)):
            for (pa, pi) in ((-2, -2.6), (-2, 2.6), (3.5, -2.6), (3.5, 2.6), (7, 2.6), (7, -2.6),
                             (0.75, 2.6), (0.75, -2.6), (5.2, 2.6), (5.2, -2.6)):
                p = r1.a_mundo(pa, pi)
                la, li = r2.a_local(p)
                for (a0, a1, i0, i1) in PIEZAS:
                    if a0 <= la <= a1 and i0 <= li <= i1:
                        return True
        return False

    def _empujar(self, r, color):
        p = self.cubos[color]
        la, li = r.a_local(p)
        if la < -5 or la > 10 or abs(li) > 7:
            return
        for (a0, a1, i0, i1) in PIEZAS:
            ca = min(max(la, a0), a1)
            ci = min(max(li, i0), i1)
            da, di = la - ca, li - ci
            d = math.hypot(da, di)
            if d >= R_CUBO:
                continue
            if d < 1e-6:     # el centro quedó adentro: sacarlo por el lado más corto
                opciones = [(la - a0, -1, 0), (a1 - la, 1, 0), (li - i0, 0, -1), (i1 - li, 0, 1)]
                opciones.sort()
                pen, sa, si = opciones[0]
                na, ni, mover = sa, si, pen + R_CUBO
            else:
                na, ni, mover = da / d, di / d, R_CUBO - d
            la += na * mover
            li += ni * mover
            self.ultimo_toque[color] = r.id
            if self.borroso:
                self._tocando.add(r.id)
        nx, ny = r.a_mundo(la, li)
        p[0], p[1] = nx, ny

    def _cubos_entre_si(self):
        ks = [k for k in self.cubos if k not in self.caidos]
        for i in range(len(ks)):
            for j in range(i + 1, len(ks)):
                p, q = self.cubos[ks[i]], self.cubos[ks[j]]
                d = math.hypot(p[0] - q[0], p[1] - q[1])
                if 1e-6 < d < 2 * R_CUBO:
                    m = (2 * R_CUBO - d) / 2
                    ux, uy = (p[0] - q[0]) / d, (p[1] - q[1]) / d
                    p[0] += ux * m
                    p[1] += uy * m
                    q[0] -= ux * m
                    q[1] -= uy * m

    # -------------------------------------------------------------- árbitro
    def _arbitro(self):
        todos = True
        for color, p in self.cubos.items():
            z = ZONAS[color]
            dentro = color not in self.caidos and falta_zona(p, z) <= TOLERANCIA
            if dentro:
                if color not in self.dentro_desde:
                    self.dentro_desde[color] = self.t
                    if self.fase == "RUNNING":
                        self.tiempo_entrada[color] = (self.t - self.t_fase, self.ultimo_toque.get(color))
                self.in_depot[color] = self.t - self.dentro_desde[color] >= 1.0
            else:
                if self.in_depot.get(color):
                    self.log.append("%.1f s: cubo %s SALIO de su zona" % (self.t, color))
                self.dentro_desde.pop(color, None)
                self.tiempo_entrada.pop(color, None)
                self.in_depot[color] = False
            todos = todos and self.in_depot[color]
        if todos and self.fase == "RUNNING":
            self._fin("reto_cumplido")

    def _fin(self, motivo):
        self.fase, self.t_fase = "FINISHED", self.t
        self.fin_motivo = motivo
        self.fin_t = self.t
        if motivo == "reto_cumplido" and self.tiempo_entrada:
            self.fin_t = self.t_run + max(t for t, _ in self.tiempo_entrada.values())

    def preparar(self):
        self.fase, self.t_fase = "READY", self.t

    # --------------------------------------------------------------- cámara
    def foto(self):
        """Mensaje del contrato v3 tal como lo vería la cámara AHORA."""
        self.seq += 1
        g = self.rng.gauss if self.ruido else (lambda a, b: 0.0)
        ahora_ms = int(self.t * 1000)
        rovers = []
        for r in self.rovers.values():
            if r.caido:
                continue
            perder = self.perdida
            if self.borroso:
                # con exposición larga el marcador se borronea cuando el robot va rápido
                vel = abs(r.vl + r.vr) / 2.0 + abs(r.omega) * 0.05
                if getattr(self, "borroso_fuerte", False):
                    perder = max(perder, min(0.97, vel / 6.0))   # exposición -4 en cuarto oscuro
                else:
                    perder = max(perder, min(0.7, vel / 18.0))
            if not (-0.5 < r.x < GRID[0] + 0.5 and -0.5 < r.y < GRID[1] + 0.5):
                perder = 1.0                  # la visión descarta marcadores fuera de la cancha
            visto = self.rng.random() >= perder
            k = ("r", r.id)
            if visto:
                self.ultimo_visto[k] = (r.x + g(0, 0.06), r.y + g(0, 0.06),
                                        (r.th + self.marcador_girado.get(r.id, 0.0) + g(0, 1.5)) % 360, ahora_ms)
            if k in self.ultimo_visto:
                x, y, th, ts = self.ultimo_visto[k]
                rovers.append({"id": r.id, "col": round(x, 3), "row": round(y, 3),
                               "theta": round(th, 2), "age_ms": ahora_ms - ts})
        cubes = []
        for color, p in self.cubos.items():
            if color in self.caidos:
                continue
            tapado = False
            for r in self.rovers.values():
                la, li = r.a_local(p)
                if -2.5 <= la <= 3.0 and abs(li) <= 2.8:     # debajo del cuerpo
                    tapado = True
            k = ("c", color)
            if not tapado and self.rng.random() < self.ver_cubo.get(color, 1.0):
                e = 0.06
                if self.borroso:
                    for r in self.rovers.values():
                        la, li = r.a_local(p)
                        if -2.5 <= la <= 8.0 and abs(li) <= 4.0:
                            e = 0.6       # las paletas tapan parte del cubo: centro impreciso
                self.ultimo_visto[k] = (p[0] + g(0, e), p[1] + g(0, e), ahora_ms)
            if k in self.ultimo_visto:
                x, y, ts = self.ultimo_visto[k]
                cubes.append({"color": color, "col": round(x, 3), "row": round(y, 3),
                              "age_ms": ahora_ms - ts, "in_depot": bool(self.in_depot.get(color))})
        if self.fase == "READY":
            total = int(self.prep * 1000)
        elif self.fase in ("RUNNING", "FINISHED"):
            total = int(self.duracion * 1000)
        else:
            total = 0
        if self.fase == "FINISHED":
            trans = int(((self.fin_t or self.t) - self.t_run) * 1000)
        elif total:
            trans = int((self.t - self.t_fase) * 1000)
        else:
            trans = 0
        msg = {
            "v": 3, "seq": self.seq, "ts_ms": int(self.ts0) + ahora_ms, "phase": self.fase,
            "clock": {"elapsed_ms": trans, "remaining_ms": max(0, total - trans), "total_ms": total},
            "grid": {"cols": GRID[0], "rows": GRID[1], "cell_mm": 20.0},
            "rovers": rovers, "cubes": cubes, "obstacles": [],
            "start": {"col": SALIDA[0], "row": SALIDA[1]},
            "depots": [{"color": c, "col": z[0], "row": z[1]} for c, z in ZONAS.items()],
            "depot_size": {"length": ZONA_TAM[0], "depth": ZONA_TAM[1]},
            "cube_side": LADO_CUBO,
        }
        return json.dumps(msg, separators=(",", ":"))


def cubos_al_azar(rng, n=3):
    """Cubos repartidos al azar, lejos de las zonas, de la salida y entre sí."""
    colores = ["red", "green", "blue"][:n]
    while True:
        pos = {}
        ok = True
        for c in colores:
            for _ in range(200):
                p = (rng.uniform(8, GRID[0] - 8), rng.uniform(8, GRID[1] - 8))
                if math.hypot(p[0] - SALIDA[0], p[1] - SALIDA[1]) < 12:
                    continue
                if any(falta_zona(p, z) < 3 for z in ZONAS.values()):
                    continue
                if any(math.hypot(p[0] - q[0], p[1] - q[1]) < 6 for q in pos.values()):
                    continue
                pos[c] = p
                break
            else:
                ok = False
        if ok:
            return pos


# ---------------------------------------------------------------------------
# Cubos como los pone la organización (docs/index.html del repo oficial,
# "Generador de posición inicial de cubos"), pasados a las coordenadas de la
# visión. El generador usa un tablero de 50x50 con el área útil 5..44, los
# robots abajo, el rojo arriba, el verde a la izquierda y el azul a la derecha;
# acá eso queda girado: robots a la izquierda, rojo a la derecha, verde arriba
# y azul abajo (como en la cámara).
# ---------------------------------------------------------------------------
_G_GOALS = {"red": (21, 1, 8, 6), "green": (1, 21, 6, 8), "blue": (43, 21, 6, 8)}
_G_ROBOTS = [(19, 46, 4, 3), (27, 46, 4, 3)]


def _g_inter(a, b):
    return a[0] < b[0] + b[2] and a[0] + a[2] > b[0] and a[1] < b[1] + b[3] and a[1] + a[3] > b[1]


def _g_valido(c, otros):
    r = (c[0], c[1], 3, 3)
    if r[0] < 5 or r[1] < 5 or r[0] + 2 > 44 or r[1] + 2 > 44:
        return False
    if any(_g_inter(r, z) for z in _G_GOALS.values()) or any(_g_inter(r, z) for z in _G_ROBOTS):
        return False
    for o in otros:
        if r[0] < o[0] + 3 + 2 and r[0] + 3 + 2 > o[0] and r[1] < o[1] + 3 + 2 and r[1] + 3 + 2 > o[1]:
            return False
    return True


def _g_seg_dist(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    if ((o1 > 0 > o2) or (o1 < 0 < o2)) and ((o3 > 0 > o4) or (o3 < 0 < o4)):
        return 0.0

    def ps(p, s, t):
        dx, dy = t[0] - s[0], t[1] - s[1]
        l2 = dx * dx + dy * dy
        k = max(0.0, min(1.0, ((p[0] - s[0]) * dx + (p[1] - s[1]) * dy) / l2)) if l2 else 0.0
        return math.hypot(p[0] - (s[0] + k * dx), p[1] - (s[1] + k * dy))
    return min(ps(a, c, d), ps(b, c, d), ps(c, a, b), ps(d, a, b))


def cubos_oficiales(rng, nivel=None):
    """Como el generador oficial. nivel = dificultad 0..1 (al azar si None)."""
    if nivel is None:
        nivel = rng.random()
    keys = ["red", "blue", "green"]

    def centro_meta(k):
        g = _G_GOALS[k]
        return (g[0] + g[2] / 2.0, g[1] + g[3] / 2.0)

    mejor, mejor_l = None, 1e9
    esperado = 6.5 + 35 * nivel
    deseado = 0 if nivel < .28 else (1 if nivel < .53 else (2 if nivel < .77 else 3))
    for intento in range(700):
        wave = math.sin(math.pi * nivel)
        j = 2 if nivel < .1 else (2.4 if nivel > .9 else 3.6)
        anc = {"red": (23.5 + rng.uniform(-j, j), 8.5 + 31 * nivel + rng.uniform(-j, j)),
               "green": (8.5 + 29 * nivel + rng.uniform(-j, j), 23 + 5 * wave + rng.uniform(-j, j)),
               "blue": (38.5 - 29 * nivel + rng.uniform(-j, j), 23 - 5 * wave + rng.uniform(-j, j))}
        lay = [(int(min(max(round(anc[k][0]), 5), 42)), int(min(max(round(anc[k][1]), 5), 42)), k) for k in keys]
        if not all(_g_valido(c, [o for o in lay if o is not c]) for c in lay):
            continue
        cent = [(c[0] + 1.5, c[1] + 1.5) for c in lay]
        dist = sum(abs(cent[i][0] - centro_meta(lay[i][2])[0]) + abs(cent[i][1] - centro_meta(lay[i][2])[1])
                   for i in range(3)) / 3.0
        cruces = 0
        for a in range(3):
            for b in range(a + 1, 3):
                if _g_seg_dist(cent[a], centro_meta(lay[a][2]), cent[b], centro_meta(lay[b][2])) < 4:
                    cruces += 1
        loss = abs(dist - esperado) / 36 * .55 + abs(cruces - deseado) / 3 * .45 + rng.uniform(0, .06)
        if loss < mejor_l:
            mejor, mejor_l = lay, loss
        if mejor_l < .035 and intento > 40:
            break
    if mejor is None:
        return cubos_al_azar(rng)
    # generador (gx a la derecha, gy hacia abajo, robots abajo) -> visión.
    # Las metas del generador miden 8x6 y las de la visión 10x7,5: se ajusta
    # con los centros de las metas y de la salida (si no, quedaban cubos
    # DENTRO de una zona, cosa que el generador oficial no permite).
    res = {}
    for gx, gy, k in mejor:
        cx, cy = gx + 1.5, gy + 1.5
        res[k] = (39.25 - (cy - 4.0) * (35.5 / 43.5), 3.75 + (cx - 4.0) * (35.5 / 42.0))
    return res
