# mundo.py — interpreta UN mensaje de telemetría (CONTRATO v3) y lo deja en
# una forma cómoda para el resto del código. Sin hardware: corre igual en la
# PC (simulador) y en el robot.
#
# Reglas del contrato que se respetan acá:
#   6.1 iterar y buscar por identidad (rover por id, cubo por color)
#   6.2 age_ms alto = tapado, NO desaparecido
#   6.5 descartar versiones desconocidas
#   grid, depots, depot_size, cube_side, start: se LEEN del mensaje, nunca fijos

import json

VERSIONES_ACEPTADAS = (3,)


class Mundo:
    """Foto de la cancha en un instante. Todo en celdas / grados del contrato."""

    def __init__(self):
        self.valido = False
        self.seq = -1
        self.ts_ms = 0
        self.fase = "IDLE"
        self.transcurrido_ms = 0
        self.restante_ms = 0
        self.cols = 43
        self.rows = 43
        self.cell_mm = 20.0
        self.rovers = {}      # id -> (col, row, theta, age_ms)
        self.cubos = {}       # color -> [col, row, age_ms, in_depot]
        self.zonas = {}       # color -> (col, row) centro de la zona
        self.zona_tam = (10.0, 7.5)   # (largo, fondo) en celdas
        self.lado_cubo = 3.0
        self.salida = (3.75, 21.5)
        self.recibido_ms = 0  # reloj LOCAL del robot cuando llegó

    @property
    def grid(self):
        return (self.cols, self.rows)


def interpretar(linea, ahora_ms):
    """linea: bytes o str con UN objeto JSON. Devuelve Mundo o None si no sirve."""
    try:
        if isinstance(linea, (bytes, bytearray)):
            linea = linea.decode("utf-8")
        m = json.loads(linea)
    except (ValueError, UnicodeError):
        return None
    if not isinstance(m, dict) or m.get("v") not in VERSIONES_ACEPTADAS:
        return None
    w = Mundo()
    try:
        w.seq = m["seq"]
        w.ts_ms = m.get("ts_ms", 0)
        w.fase = m["phase"]
        reloj = m.get("clock") or {}
        w.transcurrido_ms = reloj.get("elapsed_ms", 0)
        w.restante_ms = reloj.get("remaining_ms", 0)
        g = m["grid"]
        w.cols = g["cols"]
        w.rows = g["rows"]
        w.cell_mm = g.get("cell_mm", 20.0)
        for r in m.get("rovers", ()):
            w.rovers[r["id"]] = (float(r["col"]), float(r["row"]),
                                 float(r["theta"]), r.get("age_ms", 0))
        for c in m.get("cubes", ()):
            w.cubos[c["color"]] = [float(c["col"]), float(c["row"]),
                                   c.get("age_ms", 0), bool(c.get("in_depot", False))]
        for d in m.get("depots", ()):
            w.zonas[d["color"]] = (float(d["col"]), float(d["row"]))
        ds = m.get("depot_size")
        if ds:
            w.zona_tam = (float(ds["length"]), float(ds["depth"]))
        if "cube_side" in m:
            w.lado_cubo = float(m["cube_side"])
        s = m.get("start")
        if s:
            w.salida = (float(s["col"]), float(s["row"]))
    except (KeyError, TypeError, ValueError):
        return None
    w.recibido_ms = ahora_ms
    w.valido = True
    return w
