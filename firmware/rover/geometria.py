# geometria.py — matemática pura (sin hardware). Corre igual en el robot
# (CircuitPython) y en la PC (simulador y pruebas).
#
# Convenciones del CONTRATO v3 (sección 4):
#   - posiciones en CELDAS con decimales (1 celda = 20 mm), origen = marcador 0
#   - col crece a la derecha, row crece hacia ABAJO
#   - theta en grados, 0 = derecha (col creciente), sentido ANTIHORARIO
#     => vector de avance = (cos t, -sin t)   (el menos es porque row baja)
# Los puntos se manejan como tuplas (col, row).

import math

RAD = math.pi / 180.0
DEG = 180.0 / math.pi


def dist(a, b):
    dc = b[0] - a[0]
    dr = b[1] - a[1]
    return math.sqrt(dc * dc + dr * dr)


def norm_ang(a):
    """Ángulo a (-180, 180]."""
    a = a % 360.0
    if a > 180.0:
        a -= 360.0
    return a


def dif_ang(objetivo, actual):
    """Cuánto hay que girar (grados, + = antihorario) para ir de actual a objetivo."""
    return norm_ang(objetivo - actual)


def rumbo(a, b):
    """Theta (convención del contrato) que apunta de a hacia b."""
    return (math.atan2(-(b[1] - a[1]), b[0] - a[0]) * DEG) % 360.0


def vector(theta):
    """Vector unitario de avance (dcol, drow) para un theta del contrato."""
    t = theta * RAD
    return (math.cos(t), -math.sin(t))


def mover(p, theta, d):
    """Punto a distancia d (celdas) de p en la dirección theta."""
    v = vector(theta)
    return (p[0] + v[0] * d, p[1] + v[1] * d)


def a_mundo(pose, a, i):
    """Punto que está `a` celdas adelante e `i` a la izquierda del robot."""
    v = vector(pose[2])
    return (pose[0] + v[0] * a + v[1] * i, pose[1] + v[1] * a - v[0] * i)


def a_marco_robot(pose, p):
    """Coordenadas de p en el marco del robot: (adelante, izquierda)."""
    v = vector(pose[2])
    dc = p[0] - pose[0]
    dr = p[1] - pose[1]
    adelante = dc * v[0] + dr * v[1]
    # vector "izquierda" del robot = vector(theta + 90) = (v[1], -v[0])
    izquierda = dc * v[1] - dr * v[0]
    return adelante, izquierda


def dist_punto_segmento(p, a, b):
    """Distancia mínima de p al segmento a-b."""
    ac = b[0] - a[0]
    ar = b[1] - a[1]
    l2 = ac * ac + ar * ar
    if l2 < 1e-9:
        return dist(p, a)
    t = ((p[0] - a[0]) * ac + (p[1] - a[1]) * ar) / l2
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    return dist(p, (a[0] + t * ac, a[1] + t * ar))


def limitar(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


# ---------------------------------------------------------------------------
# Zonas de acopio (CONTRATO sección 3, "Cuándo un cubo está en su zona")
# ---------------------------------------------------------------------------

def lado_de_zona(depot, grid):
    """Borde sobre el que apoya la zona: 'arriba', 'abajo', 'izquierda', 'derecha'."""
    d = {
        "arriba": depot[1],
        "abajo": grid[1] - depot[1],
        "izquierda": depot[0],
        "derecha": grid[0] - depot[0],
    }
    mejor = None
    for k in d:
        if mejor is None or d[k] < d[mejor]:
            mejor = k
    return mejor


# theta que apunta HACIA el borde (de adentro de la cancha hacia afuera)
THETA_HACIA_BORDE = {"arriba": 90.0, "abajo": 270.0, "izquierda": 180.0, "derecha": 0.0}


def falta_para_zona(cubo, depot, grid, depot_size, cube_side):
    """Copia de la cuenta oficial: cuánto le falta (celdas) al centro del cubo
    para quedar COMPLETAMENTE dentro de la zona. 0.0 = adentro.
    cubo y depot son (col,row); grid = (cols, rows); depot_size = (largo, fondo)."""
    lado = lado_de_zona(depot, grid)
    if lado in ("arriba", "abajo"):
        semi_c, semi_r = depot_size[0] / 2.0, depot_size[1] / 2.0
    else:
        semi_c, semi_r = depot_size[1] / 2.0, depot_size[0] / 2.0
    margen = cube_side * 0.70710678
    ec = max(0.0, abs(cubo[0] - depot[0]) - (semi_c - margen))
    er = max(0.0, abs(cubo[1] - depot[1]) - (semi_r - margen))
    return math.sqrt(ec * ec + er * er)


def _orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def dist_seg_seg(a, b, c, d):
    """Distancia mínima entre los segmentos a-b y c-d (0 si se cruzan)."""
    o1, o2 = _orient(a, b, c), _orient(a, b, d)
    o3, o4 = _orient(c, d, a), _orient(c, d, b)
    if ((o1 > 0) != (o2 > 0)) and ((o3 > 0) != (o4 > 0)):
        return 0.0
    return min(dist_punto_segmento(a, c, d), dist_punto_segmento(b, c, d),
               dist_punto_segmento(c, a, b), dist_punto_segmento(d, a, b))


def capsula(pose, atras, adelante):
    """Segmento que representa el largo del robot (de la cola a la punta)."""
    p = (pose[0], pose[1])
    return mover(p, pose[2], -atras), mover(p, pose[2], adelante)


def _esquinas(pose, atras, adelante, medio):
    """Puntos del contorno del robot (esquinas y medios de los costados)."""
    v = vector(pose[2])
    iz = (v[1], -v[0])
    m = (adelante - atras) / 2.0
    pts = []
    for a in (-atras, m, adelante):
        for i in (-medio, medio):
            pts.append((pose[0] + v[0] * a + iz[0] * i, pose[1] + v[1] * a + iz[1] * i))
    return pts


def _dist_a_rect(pose, p, atras, adelante, medio):
    a, i = a_marco_robot(pose, p)
    dx = max(-atras - a, 0.0, a - adelante)
    dy = max(abs(i) - medio, 0.0)
    if dx == 0.0 and dy == 0.0:
        return -min(a + atras, adelante - a, medio - abs(i))
    return math.sqrt(dx * dx + dy * dy)


def holgura_rect(pa, pb, atras, adelante, medio):
    """Espacio libre (celdas) entre dos robots modelados como rectángulos
    (de la cola a la punta de las paletas). Negativo = se tocan."""
    d = 99.0
    for p in _esquinas(pa, atras, adelante, medio):
        x = _dist_a_rect(pb, p, atras, adelante, medio)
        if x < d:
            d = x
    for p in _esquinas(pb, atras, adelante, medio):
        x = _dist_a_rect(pa, p, atras, adelante, medio)
        if x < d:
            d = x
    return d
