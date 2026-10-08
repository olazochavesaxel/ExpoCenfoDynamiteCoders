# analizar_registro.py — resume lo que pasó en una sesión del panel.
#
# El panel (herramientas/panel.py) guarda todo en registros/sesion_*.ndjson:
# la telemetría de la cámara, los informes de los robots y las órdenes de
# prueba. Este script lo lee y cuenta la historia: qué hizo cada robot, si
# corrigió el marcador, si se acercó al borde, cuántas veces frenó y por qué.
#
#   python herramientas/analizar_registro.py              (la última sesión)
#   python herramientas/analizar_registro.py registros/sesion_20261003_181500.ndjson

import glob
import json
import math
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))


def cargar(ruta):
    filas = []
    with open(ruta, encoding="utf-8") as f:
        for linea in f:
            try:
                filas.append(json.loads(linea))
            except ValueError:
                pass
    return filas


def main():
    if len(sys.argv) > 1:
        ruta = sys.argv[1]
    else:
        todas = sorted(glob.glob(os.path.join(AQUI, "..", "registros", "sesion_*.ndjson")))
        if not todas:
            sys.exit("No hay sesiones en registros/ (se crean al abrir el panel).")
        ruta = todas[-1]
    filas = cargar(ruta)
    if not filas:
        sys.exit("El registro está vacío.")
    t0 = filas[0]["t"]

    def ts(t):
        return "%6.1f s" % ((t - t0) / 1000.0)

    print("Sesión:", os.path.basename(ruta), " duración %.0f s" % ((filas[-1]["t"] - t0) / 1000.0))
    primera = {}
    for f in filas:
        if f["k"] == "tele":
            tsm = f["d"].get("ts_ms")
            if tsm is not None and tsm not in primera:
                primera[tsm] = f["t"]
    if len(primera) > 10:
        tss = sorted(primera)
        dur = (tss[-1] - tss[0]) / 1000.0
        lat = sorted(primera[t] - t for t in tss)
        print("Cámara: %.1f fotos nuevas por segundo, atraso típico %d ms%s" % (
            (len(tss) - 1) / max(dur, 0.001), lat[len(lat) // 2],
            "  <-- LENTA (lo normal son ~20)" if (len(tss) - 1) / max(dur, 0.001) < 8 else ""))
    vistos = {}
    for f in filas:
        if f["k"] == "tele":
            for c in f["d"].get("cubes", []):
                if c.get("age_ms", 9999) < 500:
                    vistos[c["color"]] = vistos.get(c["color"], 0) + 1
    n_tele = sum(1 for f in filas if f["k"] == "tele")
    if n_tele:
        print("Cubos que la cámara ve bien: " + ", ".join(
            "%s %d%%" % (c, 100 * vistos.get(c, 0) // n_tele) for c in ("red", "green", "blue")))
    print()

    # --- fases y cubos -----------------------------------------------------
    fase = None
    entregados = {}
    print("FASES Y CUBOS")
    for f in filas:
        if f["k"] != "tele":
            continue
        m = f["d"]
        if m.get("phase") != fase:
            fase = m.get("phase")
            print("  %s  fase %s" % (ts(f["t"]), fase))
        for c in m.get("cubes", []):
            if c.get("in_depot") and c["color"] not in entregados:
                entregados[c["color"]] = f["t"]
                print("  %s  cubo %s ENTREGADO" % (ts(f["t"]), c["color"]))
    print()

    # --- por robot ---------------------------------------------------------
    for rid in (10, 11):
        inf = [f for f in filas if f["k"] == "robot" and f["d"].get("id") == rid]
        pos = []
        for f in filas:
            if f["k"] == "tele":
                for r in f["d"].get("rovers", []):
                    if r["id"] == rid and r.get("age_ms", 0) < 300:
                        pos.append((f["t"], r["col"], r["row"], r["theta"]))
        if not inf and not pos:
            continue
        print("ROBOT %d" % rid)
        if pos:
            g = filas[0]["d"].get("grid", {"cols": 43, "rows": 43}) if filas[0]["k"] == "tele" else {"cols": 43, "rows": 43}
            cols, rows = g.get("cols", 43), g.get("rows", 43)
            borde = min(min(p[1], p[2], cols - p[1], rows - p[2]) for p in pos)
            afuera = sum(1 for p in pos if not (0 <= p[1] <= cols and 0 <= p[2] <= rows))
            # una muestra cada medio segundo (si no, el ruido de la cámara suma)
            muestras = [pos[0]]
            for p in pos:
                if p[0] - muestras[-1][0] >= 500:
                    muestras.append(p)
            dist = 0.0
            giro = 0.0
            for i in range(1, len(muestras)):
                a, b = muestras[i - 1], muestras[i]
                dd = math.hypot(b[1] - a[1], b[2] - a[2])
                if dd > 0.3:
                    dist += dd
                d = (b[3] - a[3] + 540) % 360 - 180
                if abs(d) > 3:
                    giro += abs(d)
            print("  recorrió %.0f celdas (%.1f m), giró %.0f grados en total" % (dist, dist * 0.02, giro))
            print("  lo más cerca del borde: %.1f celdas%s" % (
                borde, "  <-- SE SALIÓ de la línea de los marcadores" if borde < 0 else ""))
            if afuera:
                print("  cuadros con el centro fuera de la cancha: %d" % afuera)
        if inf:
            dm = [f["d"].get("dm") for f in inf if f["d"].get("dm") is not None]
            hz = [f["d"].get("hz") for f in inf if f["d"].get("hz")]
            if dm:
                print("  desfase del marcador: empezó en %s°, terminó en %s°" % (dm[0], dm[-1]))
            if hz:
                print("  vueltas por segundo: mínimo %d, promedio %.0f%s" % (
                    min(hz), sum(hz) / len(hz), "  <-- LENTO" if min(hz) < 12 else ""))
            # --- conexión: silencios, SIN VISIÓN, señal, reinicios
            huecos = []
            for a, b in zip(inf, inf[1:]):
                if b["t"] - a["t"] > 1500:
                    huecos.append((a["t"], (b["t"] - a["t"]) / 1000.0))
            sv = [f for f in inf if f["d"].get("e") == "SIN_VISION"]
            rssi = [f["d"]["rssi"] for f in inf if isinstance(f["d"].get("rssi"), (int, float))]
            rr = sorted(set(str(f["d"].get("rr")) for f in inf if f["d"].get("rr")))
            errs = []
            for f in inf:
                e = f["d"].get("err")
                if e and (not errs or errs[-1][1] != e):
                    errs.append((f["t"], e))
            if rssi:
                print("  señal WiFi: peor %d dBm, promedio %.0f dBm%s" % (
                    min(rssi), sum(rssi) / len(rssi), "  <-- DÉBIL" if min(rssi) < -75 else ""))
            if rr:
                print("  motivo del último reinicio de la placa:", ", ".join(rr),
                      "  <-- se reinició sola" if any(x not in ("POWER_ON", "None", "?") for x in rr) else "")
            if huecos:
                print("  silencios (el robot no mandó nada más de 1,5 s): %d, el peor de %.1f s" % (
                    len(huecos), max(h[1] for h in huecos)))
                for t_, d_ in huecos[:8]:
                    print("    %s  %.1f s callado" % (ts(t_), d_))
            if sv:
                print("  informes SIN VISIÓN: %d (unos %.0f s en total)" % (len(sv), len(sv) * 0.2))
            for t_, e in errs[:10]:
                print("    %s  error de conexión: %s" % (ts(t_), e))
            frenos = {}
            for f in inf:
                k = f["d"].get("f")
                if k:
                    frenos[k] = frenos.get(k, 0) + 1
            if frenos:
                print("  frenadas (informes, 5 por s):", ", ".join("%s x%d" % kv for kv in sorted(frenos.items())))
            print("  historia:")
            ult = None
            vistas = set()
            for f in inf:
                d = f["d"]
                for n, txt in d.get("lg") or []:
                    if n not in vistas:
                        vistas.add(n)
                        print("    %s  %s" % (ts(f["t"]), txt))
                clave = (d.get("e"), d.get("t"))
                if clave != ult and not d.get("lg"):
                    print("    %s  estado %s %s" % (ts(f["t"]), d.get("e"), d.get("t") or ""))
                ult = clave
        print()

    ords = [f for f in filas if f["k"] == "orden"]
    if ords:
        print("ÓRDENES DEL PANEL")
        for f in ords:
            print("  %s  robot %s <- %s" % (ts(f["t"]), f["d"]["id"], f["d"]["txt"]))


if __name__ == "__main__":
    main()
