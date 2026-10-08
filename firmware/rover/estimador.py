# estimador.py — rumbo del robot: giroscopio (rápido) + cámara (exacta).

import math
import geometria as G


class Estimador:
    """Dónde está y hacia dónde mira el robot AHORA.

    La cámara es exacta pero lenta y atrasada: en una compu común manda una
    imagen nueva cada ~0,4 s y llega ~0,35 s tarde (medido en casa, oct-2026).
    Entre imagen e imagen el robot se mueve mucho, así que:
      - el RUMBO sale del giroscopio (instantáneo) y la cámara solo corrige
        su deriva, comparando cada foto con lo que decía el giroscopio EN EL
        MOMENTO de esa foto (no ahora);
      - la POSICIÓN es la de la última foto más lo que el robot avanzó desde
        entonces según los motores (odometría), con una escala que se ajusta
        sola comparando con la cámara;
      - el ATRASO de la cámara se mide solo, comparando los giros que ve la
        cámara con los del giroscopio.
    Sin giroscopio funciona igual pero peor (rumbo = cámara)."""

    LATENCIAS = tuple(range(0, 1000, 50))      # candidatos de atraso (ms)
    LENTO = 0.2                                # potencia útil por debajo = "andar lento"

    def __init__(self, latencia_ms=350, escala=30.0):
        self.hg = 0.0          # rumbo integrado del giroscopio (grados)
        self.off = 0.0         # corrección: rumbo real = hg + off
        self.ini = False
        self.vel = 0.0         # grados/s (antihorario +)
        self.con_giro = False
        self.th_vision = 0.0
        # verificación del signo del giroscopio (GIRO_SIGNO ya se midió en la
        # prueba sin cámara, así que se confía desde el arranque)
        self._prev = None
        self.signo_s = 0.0
        self.signo_tot = 0.0
        self.signo_invertido = False
        self.signo_ok = True
        self._grande = 0
        # odometría
        self.dx = 0.0          # desplazamiento acumulado según los motores (celdas)
        self.dy = 0.0
        # celdas/s por unidad de potencia útil, aprendida APARTE para cada forma
        # de andar: los motores con poca potencia (empujando un cubo, batería
        # floja) rinden mucho menos que a velocidad de crucero, y marcha atrás
        # es otra cosa. Medido en casa: 1,4 celdas/s con v=0,13 adelante y 5
        # celdas/s con v=-0,13 atrás. Con UNA sola escala el robot creía que
        # avanzaba el triple y soltaba el cubo antes de llegar.
        # Arranca suponiendo que es RÁPIDO (pesimista): si en realidad va más
        # lento, se para antes de tiempo y espera la foto (no pasa nada); si
        # creyera que es lento y fuera rápido, se pasaría y tiraría el cubo.
        self.escalas = [escala * 1.3, escala * 1.3, escala * 1.3]   # [lento, rápido, atrás]
        self._acum = [0.0, 0.0, 0.0]
        self._umin = 9.0          # potencia mínima y máxima desde la última foto:
        self._umax = -9.0         # solo se aprende si anduvo PAREJO (sin acelerar ni frenar)
        self.base = None       # (col, row, dx, dy) de la última foto
        self.hist = []         # [(ms, hg, dx, dy)] últimos ~2 s
        self._ult_hist = -1000
        # atraso de la cámara
        self.latencia = float(latencia_ms)
        self._err = [0.0] * len(self.LATENCIAS)
        self._n_lat = 0
        self._foto_prev = None   # (ms recibida, theta)
        self._cam_prev = None    # (ms, col, row, dx, dy) para ajustar la escala

    # ------------------------------------------------------- integración
    def giroscopio(self, dps, dt):
        if dps is None:
            self.con_giro = False
            return
        self.con_giro = True
        if self.signo_invertido:
            dps = -dps
        self.vel = dps
        self.hg += dps * dt

    def avanzar(self, potencia_util, dt):
        """potencia_util: promedio de las dos ruedas por encima de la zona muerta."""
        if potencia_util == 0.0 or dt <= 0:
            return
        b = 2 if potencia_util < 0 else (0 if potencia_util < self.LENTO else 1)
        d = self.escalas[b] * potencia_util * dt
        self._acum[b] += abs(d)
        if potencia_util < self._umin:
            self._umin = potencia_util
        if potencia_util > self._umax:
            self._umax = potencia_util
        v = G.vector(self.rumbo())
        self.dx += v[0] * d
        self.dy += v[1] * d

    def registrar(self, ahora):
        if ahora - self._ult_hist < 40:
            return
        self._ult_hist = ahora
        self.hist.append((ahora, self.hg, self.dx, self.dy))
        if len(self.hist) > 55:
            self.hist.pop(0)

    def _en(self, t):
        """(hg, dx, dy) en el instante t (ms), interpolando la historia."""
        h = self.hist
        if not h or t >= h[-1][0]:
            return self.hg, self.dx, self.dy
        if t <= h[0][0]:
            return h[0][1], h[0][2], h[0][3]
        for k in range(len(h) - 1, 0, -1):
            a = h[k - 1]
            if a[0] <= t:
                b = h[k]
                f = (t - a[0]) / float(b[0] - a[0]) if b[0] > a[0] else 0.0
                return (a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f,
                        a[3] + (b[3] - a[3]) * f)
        return h[0][1], h[0][2], h[0][3]

    # ------------------------------------------------------------ cámara
    def foto(self, col, row, theta, ahora, edad_ms):
        """Una foto NUEVA de la cámara (no las copias repetidas de la misma).
        ahora: reloj del robot al recibirla; edad_ms: la que publica la visión."""
        self.th_vision = theta
        t_rec = ahora - edad_ms
        self._medir_latencia(t_rec, theta)
        t = t_rec - self.latencia                 # cuándo se sacó la foto
        hg_t, dx_t, dy_t = self._en(t)
        # posición: foto + lo que avanzó desde entonces
        ac = self._acum
        tot = ac[0] + ac[1] + ac[2]
        # ¿estaba QUIETO cuando se sacó esta foto? (entonces la foto es nítida y
        # no importa si el atraso estimado está un poco mal)
        a0 = self._en(t - 300)
        quieto = abs(a0[1] - dx_t) + abs(a0[2] - dy_t) < 0.15
        if self._cam_prev is not None:
            c0 = self._cam_prev
            cx, cy = col - c0[1], row - c0[2]
            ox, oy = dx_t - c0[3], dy_t - c0[4]
            o2 = ox * ox + oy * oy
            b = 0 if ac[0] >= ac[1] and ac[0] >= ac[2] else (1 if ac[1] >= ac[2] else 2)
            if o2 > 0.8 and (t - c0[0]) < 4000 and tot > 0 and ac[b] > 0.7 * tot and quieto and c0[5]:
                r = (cx * ox + cy * oy) / o2           # cuánto se movió de verdad / lo que creía
                if -0.2 < r < 5.0:
                    r = min(1.4, max(0.75, max(r, 0.05) ** 0.7))
                    self.escalas[b] = min(120.0, max(4.0, self.escalas[b] * r))
        if quieto:
            self._acum = [0.0, 0.0, 0.0]
        if quieto or self._cam_prev is None or not self._cam_prev[5]:
            self._cam_prev = (t, col, row, dx_t, dy_t, quieto)
        self.base = (col, row, dx_t, dy_t)
        # rumbo
        if not self.con_giro:
            return
        if not self.ini:
            self.off = theta - hg_t
            self.ini = True
            self._prev = (theta, hg_t)
            return
        if self._prev is not None:
            dv = G.dif_ang(theta, self._prev[0])
            dg = hg_t - self._prev[1]
            if abs(dg) > 5.0 and abs(dv) < 120:
                self.signo_s += dv * dg
                self.signo_tot += abs(dv * dg)
                if self.signo_tot > 3000 and self.signo_s < -0.6 * self.signo_tot:
                    self.signo_invertido = not self.signo_invertido
                    self.signo_s = 0.0
                    self.signo_tot = 0.0
                    self.ini = False
                    self._prev = None
                    return
                if self.signo_tot > 3000:
                    self.signo_s *= 0.5
                    self.signo_tot *= 0.5
        self._prev = (theta, hg_t)
        e = G.dif_ang(theta, hg_t + self.off)
        if abs(e) > 35.0:
            self._grande += 1
            if self._grande >= 3:              # tres fotos seguidas lo contradicen
                self.off += e
                self._grande = 0
            return
        self._grande = 0
        self.off += 0.35 * e

    def _medir_latencia(self, t_rec, theta):
        p = self._foto_prev
        if p is not None and t_rec - p[0] < 250:
            return                 # con cámara rápida alcanza con ~4 mediciones por segundo
        self._foto_prev = (t_rec, theta)
        if p is None or not self.con_giro or t_rec - p[0] > 1200:
            return
        dv = G.dif_ang(theta, p[1])
        if abs(dv) < 4.0 or abs(dv) > 150.0:
            return
        for k, L in enumerate(self.LATENCIAS):
            dg = self._en(t_rec - L)[0] - self._en(p[0] - L)[0]
            x = dv - dg
            self._err[k] = 0.97 * self._err[k] + x * x
        self._n_lat += 1
        if self._n_lat >= 12:
            mejor = min(range(len(self.LATENCIAS)), key=lambda k: self._err[k])
            self.latencia += 0.3 * (self.LATENCIAS[mejor] - self.latencia)

    @property
    def escala(self):
        return self.escalas[1]

    def posicion(self):
        """(col, row) estimada AHORA, o None si todavía no hubo foto."""
        b = self.base
        if b is None:
            return None
        return (b[0] + self.dx - b[2], b[1] + self.dy - b[3])

    def reiniciar_rumbo(self):
        """La cámara cambió de referencia (se corrigió el desfase del marcador)."""
        self.ini = False
        self._prev = None
        self._grande = 0

    def rumbo(self):
        if self.con_giro and self.ini and self.signo_ok:
            return (self.hg + self.off) % 360.0
        return self.th_vision


class CalibradorMarcador:
    """Mide si el marcador ArUco está pegado girado respecto de las paletas.

    La visión publica hacia dónde apunta el BORDE SUPERIOR del marcador. Si el
    marcador se pegó girado (por ejemplo, con la parte de arriba hacia el
    costado del robot), el robot cree que mira hacia un lado y en realidad
    mira hacia otro: da vueltas alrededor del cubo sin llegar, o se va hacia
    el borde. Acá se compara, mientras el robot AVANZA, la dirección en que
    realmente se desplaza (según la cámara) con la que dice el marcador.
    La diferencia es el desfase, y se corrige solo.
    """

    TRAMO = 2.5          # celdas recorridas para hacer una medición
    MAX_S = 3.5          # si tarda más, se descarta el tramo
    MAX_GIRO = 50.0      # grados de giro máximos durante el tramo

    def __init__(self, desfase=0.0):
        self.desfase = desfase
        self.mediciones = 0
        self.ultimo_error = None
        self.correcciones = 0
        self.confirmado = False   # ya hubo un tramo en que el desfase dio bien
        self._ini = None          # (ms, col, row, signo, rumbo_giro)
        self._sx = 0.0
        self._sy = 0.0
        self._pend = None         # error grande esperando confirmación

    def _cortar(self):
        self._ini = None

    def muestra(self, ahora, col, row, theta, v, rumbo_giro=None, confiar=False):
        """theta: de la cámara YA corregido con self.desfase. v: avance mandado
        a los motores (+ adelante, - atrás). confiar: el robot está yendo recto
        a propósito para medir (basta un tramo). Devuelve True si cambió el desfase."""
        signo = 1 if v > 0.08 else (-1 if v < -0.08 else 0)
        if signo == 0:
            self._cortar()
            return False
        if self._ini is None or self._ini[3] != signo:
            self._ini = (ahora, col, row, signo, rumbo_giro)
            self._sx = 0.0
            self._sy = 0.0
        t = theta * G.RAD
        self._sx += math.cos(t)
        self._sy += math.sin(t)
        t0, c0, r0, s0, g0 = self._ini
        if (ahora - t0) / 1000.0 > self.MAX_S:
            self._ini = (ahora, col, row, signo, rumbo_giro)
            self._sx = self._sy = 0.0
            return False
        if g0 is not None and rumbo_giro is not None and abs(G.dif_ang(rumbo_giro, g0)) > self.MAX_GIRO:
            self._ini = (ahora, col, row, signo, rumbo_giro)
            self._sx = self._sy = 0.0
            return False
        if G.dist((c0, r0), (col, row)) < self.TRAMO:
            return False
        # tramo completo: ¿hacia dónde se movió de verdad?
        real = G.rumbo((c0, r0), (col, row))
        if signo < 0:
            real += 180.0
        medio = math.atan2(self._sy, self._sx) * G.DEG
        e = G.dif_ang(real, medio)
        self.mediciones += 1
        self.ultimo_error = e
        self._ini = (ahora, col, row, signo, rumbo_giro)
        self._sx = self._sy = 0.0
        if abs(e) <= 15.0:
            self._pend = None
            self.confirmado = True
        if not confiar:
            # andando normalmente solo se MIDE (el panel lo muestra). Con la
            # cámara atrasada, corregir solo puede equivocarse: el desfase se
            # corrige únicamente con "Calibrar marcador" o en config.py
            return False
        if abs(e) <= 15.0:
            if abs(e) > 3.0:
                self.desfase = (self.desfase + 0.5 * e) % 360.0
            return False
        # error grande: se corrige recién cuando dos tramos seguidos coinciden
        if confiar:
            self.desfase = (self.desfase + e) % 360.0
            self.correcciones += 1
            self._pend = None
            return True
        if self._pend is not None and ahora - self._pend[0] < 8000 and abs(G.dif_ang(e, self._pend[1])) < 30.0:
            prom = self._pend[1] + G.dif_ang(e, self._pend[1]) / 2.0
            self.desfase = (self.desfase + prom) % 360.0
            self.correcciones += 1
            self._pend = None
            return True
        self._pend = (ahora, e)
        return False
