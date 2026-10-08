# plan.py — decide QUÉ cubo lleva cada rover y en qué orden, y POR DÓNDE ir.
# Matemática pura: corre igual en el robot y en el simulador de la PC.
#
# Reparto: se prueban TODAS las formas de repartir los cubos pendientes entre
# los dos rovers (con 3 cubos son pocas) y se elige la que termina antes
# (el "makespan": el tiempo del rover que más tarda). Así se minimiza el
# tiempo del último cubo, que es lo que define la clasificación (regla 22).
#
# Regla 12.2.13: para que los 3 cubos cuenten, CADA rover tiene que haber
# llevado al menos uno. El reparto lo obliga mientras haga falta.
#
# Los dos rovers corren este mismo cálculo con los mismos datos de la cámara
# (redondeados para que el ruido no cambie la respuesta), así que llegan al
# mismo reparto aunque se pierda la radio. Si la radio anda, el rover de ID
# menor manda su plan y el otro lo usa.

import geometria as G


def _permutaciones(xs):
    if len(xs) <= 1:
        return [list(xs)]
    res = []
    for i in range(len(xs)):
        resto = xs[:i] + xs[i + 1:]
        for p in _permutaciones(resto):
            res.append([xs[i]] + p)
    return res


def destino_cubo(w, color):
    """Centro de la zona del cubo."""
    return w.zonas[color]


def objetivo_zona(cfg, w, color):
    """Dónde dejar el CENTRO del cubo. Normalmente el centro de la zona; si
    otro cubo estorba el carril de entrada, nos corremos a lo largo de la zona
    (el cubo puede caer hasta ~57 mm a cada lado del centro y sigue contando)."""
    z = w.zonas[color]
    th = theta_entrada(w, color)
    a_lo_largo = th + 90.0
    # (con margen: el cubo puede quedar ~1 celda corrido entre las paletas)
    margen_largo = max(0.0, w.zona_tam[0] / 2.0 - w.lado_cubo * 0.70710678 - 1.3)
    otros = [(c[0], c[1]) for k, c in w.cubos.items() if k != color]
    mejor = None
    for f in (0.0, 0.5, -0.5, 1.0, -1.0):
        t = G.mover(z, a_lo_largo, f * margen_largo)
        a = G.mover(t, th + 180.0, cfg.AGARRE + cfg.APROX_ZONA)
        b = G.mover(t, th, 1.0)
        libre = 99.0
        for q in otros:
            d = G.dist_punto_segmento(q, a, b)
            if d < libre:
                libre = d
        if libre >= cfg.MEDIO_ANCHO + 2.0:
            return t
        if mejor is None or libre > mejor[0]:
            mejor = (libre, t)
    return mejor[1]


def theta_entrada(w, color):
    """Rumbo con el que conviene entrar a la zona: mirando hacia su borde."""
    lado = G.lado_de_zona(w.zonas[color], w.grid)
    return G.THETA_HACIA_BORDE[lado]


def _r(x):
    return int(x + 0.5)


def _tiempo_tarea(cfg, pos, rumbo_actual, cubo, zona, th_in):
    """Tiempo estimado (s) para: ir detrás del cubo, agarrarlo y dejarlo en la zona."""
    v = cfg.CELDAS_POR_S
    vr = cfg.GRADOS_POR_S
    # Punto previo a la zona y dirección de captura
    pre_zona = G.mover(zona, th_in + 180.0, cfg.APROX_ZONA)
    u = G.rumbo(cubo, pre_zona)
    pre = G.mover(cubo, u + 180.0, cfg.AGARRE + cfg.PRE_AGARRE)
    ida = G.dist(pos, pre)
    giro1 = abs(G.dif_ang(G.rumbo(pos, pre), rumbo_actual)) + abs(G.dif_ang(u, G.rumbo(pos, pre)))
    llevar = G.dist(cubo, pre_zona) + cfg.APROX_ZONA
    giro2 = abs(G.dif_ang(th_in, u))
    t = (ida + cfg.PRE_AGARRE) / v + llevar / (v * 0.8) + (giro1 + giro2) / vr + 2.0
    fin = G.mover(zona, th_in + 180.0, cfg.AGARRE + cfg.RETROCESO)
    return t, fin, th_in


def bloqueos(cfg, w, color, objetivo, otros):
    """Cubos (de 'otros') que estorban la entrada a la zona de 'color'."""
    th = theta_entrada(w, color)
    cw = w.cubos[color]
    a = G.mover(objetivo, th + 180.0, cfg.AGARRE + cfg.APROX_ZONA + 2.0)
    b = G.mover(objetivo, th, 1.0)
    res = []
    for k in otros:
        c = w.cubos[k]
        if G.dist_punto_segmento((c[0], c[1]), a, b) < cfg.MEDIO_ANCHO + 2.0:
            # cerca de la zona es grave (el robot lo arrastra adentro con el
            # suyo); al principio del carril la ruta lo puede rodear
            atras = -G.a_marco_robot((objetivo[0], objetivo[1], th), (c[0], c[1]))[0]
            res.append((k, 40.0 if atras < cfg.AGARRE + cfg.APROX_ZONA + 4.0 else 12.0))
        elif G.dist_punto_segmento((c[0], c[1]), (cw[0], cw[1]), a) < cfg.MEDIO_ANCHO + 3.0:
            # queda en el CAMINO de este cubo a su zona: llevándolo, el robot
            # no puede esquivar bien (gira abierto) y lo barre. Mejor moverlo antes.
            res.append((k, 20.0))
    return res


def repartir(cfg, w, pendientes, rovers, obligatorios, fijos):
    """
    pendientes:   lista de colores por entregar
    rovers:       {id: (col, row, theta)} de los rovers disponibles
    obligatorios: ids que todavía tienen que llevar al menos un cubo
    fijos:        {id: color} cubo que ese rover YA tiene agarrado (no se mueve)
    Devuelve {id: [colores en orden]} o None si no hay nada que hacer.
    """
    ids = sorted(rovers.keys())
    cols = sorted(pendientes)
    if not ids or not cols:
        return None
    # datos redondeados: los dos rovers tienen que llegar al mismo resultado
    pos = {}
    for i in ids:
        r = rovers[i]
        pos[i] = (_r(r[0]), _r(r[1]), 15 * _r(r[2] / 15.0))
    cub = {}
    for c in cols:
        cw = w.cubos[c]
        cub[c] = (_r(cw[0]), _r(cw[1]))

    objs = {}
    for c in cols:
        objs[c] = objetivo_zona(cfg, w, c)
    bloquea = {}
    for c in cols:
        bloquea[c] = bloqueos(cfg, w, c, objs[c], [x for x in cols if x != c])
    mejor = None
    n = len(cols)
    k = len(ids)
    for codigo in range(k ** n):
        asign = {}
        x = codigo
        for c in cols:
            asign.setdefault(ids[x % k], []).append(c)
            x //= k
        # restricciones
        ok = True
        for i, c in fijos.items():
            if c in cols and c not in asign.get(i, ()):
                ok = False
        faltan = [i for i in obligatorios if i in ids]
        if len(faltan) <= n:
            for i in faltan:
                if not asign.get(i):
                    ok = False
        if not ok:
            continue
        total = 0.0
        peor = 0.0
        orden = {}
        tramos = {}
        for i, lista in asign.items():
            mejor_i = None
            for perm in _permutaciones(lista):
                if i in fijos and fijos[i] in perm and perm[0] != fijos[i]:
                    continue
                p = (pos[i][0], pos[i][1])
                th = pos[i][2]
                t = 0.0
                hechos = []
                for c in perm:
                    dt, p, th = _tiempo_tarea(cfg, p, th, cub[c], objs[c], theta_entrada(w, c))
                    t += dt
                    for b, pena in bloquea[c]:
                        if b not in hechos:
                            t += pena          # otro cubo tapa la entrada: conviene moverlo antes
                    hechos.append(c)
                if mejor_i is None or t < mejor_i[0]:
                    mejor_i = (t, perm)
            orden[i] = mejor_i[1]
            if mejor_i[1]:
                c0 = mejor_i[1][0]
                z = objs[c0]
                tramos[i] = ((pos[i][0], pos[i][1]), cub[c0], G.mover(z, theta_entrada(w, c0) + 180.0, cfg.AGARRE))
            total += mejor_i[0]
            if mejor_i[0] > peor:
                peor = mejor_i[0]
        puntaje = peor + 0.15 * total
        # penalidad si los primeros trabajos de los dos rovers se cruzan
        if len(tramos) == 2:
            a, b = list(tramos.values())
            dmin = 1e9
            for s1 in ((a[0], a[1]), (a[1], a[2])):
                for s2 in ((b[0], b[1]), (b[1], b[2])):
                    dmin = min(dmin, G.dist_seg_seg(s1[0], s1[1], s2[0], s2[1]))
            if dmin < 10.0:
                puntaje += (10.0 - dmin) * 1.2
        clave = (round(puntaje, 1), str(sorted(orden.items())))
        if mejor is None or clave < mejor[0]:
            mejor = (clave, orden)
    if mejor is None:
        return None
    res = {}
    for i in ids:
        res[i] = mejor[1].get(i, [])
    return res


# ---------------------------------------------------------------------------
# Rutas: línea recta, y si algo estorba, un desvío por el costado más corto.
# ---------------------------------------------------------------------------

def _dentro(cfg, w, p):
    m = cfg.MARGEN_BORDE
    return (G.limitar(p[0], m, w.cols - m), G.limitar(p[1], m, w.rows - m))


def _libre(a, b, obstaculos, ignorar):
    for n, (c, r) in enumerate(obstaculos):
        if n in ignorar:
            continue
        if G.dist_punto_segmento(c, a, b) < r:
            return False
    return True


def ruta(cfg, w, a, b, obstaculos, prof=0):
    """Camino más corto de a a b que no atraviesa los círculos (centro, radio).
    Grafo de visibilidad: puntos alrededor de cada obstáculo + Dijkstra.
    Pocos obstáculos => pocos puntos: anda rápido también en el ESP32."""
    # obstáculos que ya nos tocan al salir o al llegar no se pueden esquivar
    ignorar = []
    achicar = False
    for n, (c, r) in enumerate(obstaculos):
        db = G.dist(b, c)
        da = G.dist(a, c)
        if da < r * 0.6 or db < r * 0.8:
            ignorar.append(n)
        elif db < r + 0.3 or da < r + 0.3:
            achicar = True
    if achicar:
        # el destino queda apenas dentro de un obstáculo: se achica ese círculo
        # (si no, no hay camino y el robot iba derecho, arrastrando el cubo)
        nuevos = []
        for n, (c, r) in enumerate(obstaculos):
            if n not in ignorar:
                r = min(r, G.dist(b, c) - 0.3, G.dist(a, c) - 0.3)
            nuevos.append((c, r))
        obstaculos = nuevos
    if _libre(a, b, obstaculos, ignorar):
        return [b]
    m = cfg.MARGEN_BORDE
    nodos = [a, b]
    for n, (c, r) in enumerate(obstaculos):
        if n in ignorar:
            continue
        rr = r * 1.12 + 0.3
        for k in range(8):
            q = G.mover(c, k * 45.0 + 22.5, rr)
            if not (m <= q[0] <= w.cols - m and m <= q[1] <= w.rows - m):
                continue
            dentro = False
            for j, (c2, r2) in enumerate(obstaculos):
                if j not in ignorar and G.dist(q, c2) < r2:
                    dentro = True
                    break
            if not dentro:
                nodos.append(q)
    N = len(nodos)
    INF = 1e9
    dist = [INF] * N
    previo = [-1] * N
    hecho = [False] * N
    dist[0] = 0.0
    for _ in range(N):
        u = -1
        mejor = INF
        for i in range(N):
            if not hecho[i] and dist[i] < mejor:
                mejor, u = dist[i], i
        if u < 0 or u == 1:
            break
        hecho[u] = True
        for v in range(N):
            if hecho[v] or v == u:
                continue
            d = dist[u] + G.dist(nodos[u], nodos[v])
            if d < dist[v] and _libre(nodos[u], nodos[v], obstaculos, ignorar):
                dist[v] = d
                previo[v] = u
    if dist[1] >= INF:
        return [b]                     # no hay camino libre: vamos derecho
    camino = []
    i = 1
    while i > 0:
        camino.append(nodos[i])
        i = previo[i]
    camino.reverse()
    return camino


_DELTAS = (0, 30, -30, 60, -60, 90, -90, 135, -135, 180)


def punto_previo(cfg, w, color, obstaculos, evitar=None, preferido=None):
    """
    Dónde pararse para agarrar el cubo 'color', y con qué rumbo (u).
    Preferimos agarrarlo mirando hacia la entrada de su zona, así se lleva
    derecho. Si ese punto queda fuera de la cancha o encima de otro cubo,
    probamos otros ángulos.
    """
    cubo = (w.cubos[color][0], w.cubos[color][1])
    zona = objetivo_zona(cfg, w, color)
    th_in = theta_entrada(w, color)
    pre_zona = G.mover(zona, th_in + 180.0, cfg.APROX_ZONA)
    if G.dist(cubo, pre_zona) < 2.0:
        u0 = th_in
    else:
        u0 = G.rumbo(cubo, pre_zona)
    # el punto previo no puede estar pegado al borde: al llegar ahí las
    # paletas quedarían fuera del tablero y el freno del borde no deja llegar
    mejor = None
    # el lado que ya venía usando va PRIMERO: si sigue sirviendo, no cambio de
    # idea (en las pruebas el robot iba y venía entre dos lados del cubo)
    cands = [(max(cfg.MARGEN_BORDE, cfg.PUNTA - 1.0), d) for d in _DELTAS] + \
        [(cfg.MARGEN_BORDE, d) for d in _DELTAS]
    if preferido is not None:
        cands = [(cfg.MARGEN_BORDE, G.dif_ang(preferido, u0))] + cands
    for m, delta in cands:
        u = u0 + delta
        if evitar and any(abs(G.dif_ang(u, x)) < 25.0 for x in evitar):
            continue                       # por ahí ya me trabé
        p = G.mover(cubo, u + 180.0, cfg.AGARRE + cfg.PRE_AGARRE)
        if not (m <= p[0] <= w.cols - m and m <= p[1] <= w.rows - m):
            continue
        libre = True
        holgura = 99.0
        tope = G.mover(cubo, u + 180.0, cfg.AGARRE)       # donde queda el robot al agarrarlo
        for (c, r) in obstaculos:
            h = min(G.dist(p, c) - r, G.dist_punto_segmento(c, p, tope) - r * 0.7)
            if h < 0.0:
                libre = False
            if h < holgura:
                holgura = h
        if libre:
            return p, u % 360.0
        if mejor is None or holgura > mejor[0]:
            mejor = (holgura, p, u % 360.0)
    # nada perfecto: el lado menos malo (el más lejos de lo que estorba)
    if mejor is not None:
        return mejor[1], mejor[2]
    return _dentro(cfg, w, G.mover(cubo, u0 + 180.0, cfg.AGARRE + cfg.PRE_AGARRE)), u0 % 360.0
