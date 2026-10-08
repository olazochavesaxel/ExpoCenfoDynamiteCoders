# coordinacion.py — todo lo que tiene que ver con el COMPAÑERO: radio
# (ESP-NOW), reparto de cubos, "corredores" reservados para no cruzarse,
# quién tiene preferencia, y cómo correrse del camino. Es parte de Cerebro
# (se separó en otro archivo para que el ESP32 lo compile por partes).

import json
import geometria as G
import plan as P
from estados import CON_CUBO, ACTIVOS, RANGO


def _json(m):
    try:
        return json.dumps(m, separators=(",", ":"))
    except TypeError:          # por si esta versión de CircuitPython no acepta separators
        return json.dumps(m)


class Coordinacion:

    def id_par(self):
        if self.w is None:
            return None
        for i in self.w.rovers:
            if i != self.id:
                return i
        if self.par:
            return self.par.get("id")
        return None

    def pose_par(self):
        # 1) lo que el compañero dice por radio de sí mismo: es su estimación
        #    AHORA (con giroscopio y odometría), mucho más fresca que la foto
        rp = getattr(self, "_par_pose", None)
        i = self.id_par()
        if i is None or i not in self.w.rovers or self.w.rovers[i][3] > 6000:
            # la cámara no lo ve hace rato: está fuera de la cancha (lo
            # levantaron pero sigue prendido mandando su posición por radio)
            return None
        if rp is not None and self._ahora - rp[3] < 400:
            self._par_radio = True
            return (rp[0], rp[1], rp[2], self._ahora - rp[3])
        self._par_radio = False
        # 2) si no, la cámara (atrasada)
        r = self.w.rovers[i]
        if r[3] > 3000:
            return None
        # el marcador del compañero también puede estar girado: lo que dice su
        # radio (lo que él midió) o, si no, lo que figura en config
        d = self.desf_par()
        if d:
            return (r[0], r[1], (r[2] + d) % 360.0, r[3])
        return r

    def desf_par(self):
        if self.par is not None and "dm" in self.par:
            return self.par["dm"]
        return self.cfg.DESFASES_MARCADOR.get(self.id_par(), 0.0) \
            if hasattr(self.cfg, "DESFASES_MARCADOR") else 0.0

    def _velocidad_par(self):
        """(dcol/s, drow/s, grados/s) del compañero, medida con la cámara."""
        pp = self.pose_par()
        if pp is None:
            self._vp = (0.0, 0.0, 0.0)
            return self._vp
        t = self._ahora - pp[3]
        prev = getattr(self, "_pp_prev", None)
        if prev is not None and (pp[0], pp[1]) == (prev[0], prev[1]):
            return self._vp          # misma foto repetida: nada nuevo
        if prev is not None and t - prev[3] > 80:
            dt = (t - prev[3]) / 1000.0
            if dt < 1.0:
                nv = ((pp[0] - prev[0]) / dt, (pp[1] - prev[1]) / dt, G.dif_ang(pp[2], prev[2]) / dt)
                a = 0.5
                self._vp = tuple(a * nv[k] + (1 - a) * self._vp[k] for k in range(3))
            self._pp_prev = (pp[0], pp[1], pp[2], t)
        elif prev is None:
            self._pp_prev = (pp[0], pp[1], pp[2], t)
        return self._vp

    def _holgura(self, pose_a, pose_b):
        cfg = self.cfg
        return G.holgura_rect(pose_a, pose_b, cfg.COLA, cfg.PUNTA, cfg.MEDIO_ANCHO_PAR)

    def _riesgo(self, pose, v, w, completo=False):
        """Menor holgura (celdas) prevista entre los dos robots en el próximo
        ~0,8 s si yo hago (v, w) y el otro sigue como viene."""
        cfg = self.cfg
        pp = self.pose_par()
        if pp is None:
            return 99.0
        vp = self._vp
        if vp[0] * vp[0] + vp[1] * vp[1] < 1.0:
            vp = (0.0, 0.0, 0.0)
        vel, giro = self._velocidades(v, w)
        # la posición del compañero viene de una foto vieja: se la adelanta
        if getattr(self, "_par_radio", False):
            atraso = 0.1
        else:
            atraso = min(0.8, self.est.latencia / 1000.0 + 0.15)
        peor = self._holgura(pose, (pp[0] + vp[0] * atraso, pp[1] + vp[1] * atraso, pp[2]))
        tope = 999.0
        gf = getattr(self, "_giro_falta", None)
        if abs(v) <= 0.02 and gf is not None and self._ahora - gf[1] < 200 and not completo:
            tope = gf[0] + 8.0          # girando en el lugar: no más de lo que falta
        for T in (0.25, 0.5, 0.8):
            dg = giro * T * 0.6
            if dg > tope:
                dg = tope
            elif dg < -tope:
                dg = -tope
            th = pose[2] + dg
            p = G.mover((pose[0], pose[1]), (pose[2] + th) / 2.0 if abs(giro) < 1 else th, vel * T)
            yo = (p[0], p[1], th)
            ta = T + atraso
            otro = (pp[0] + vp[0] * ta, pp[1] + vp[1] * ta, pp[2])
            h = self._holgura(yo, otro)
            if h < peor:
                peor = h
        return peor

    def lo_lleva_el_par(self, color):
        """¿El compañero tiene este cubo entre sus paletas? (visto por cámara)"""
        pp = self.pose_par()
        if pp is None:
            return False
        if self.par_fresco() and self.par.get("t") == color and self.par.get("e") in CON_CUBO:
            return True
        return self.cubo_en_paletas((pp[0], pp[1], pp[2]), color, 0.5)

    def par_fresco(self, ms=1200):
        return self.par is not None and self._ahora - self.par_ms < ms

    # ------------------------------------------------------- bucle principal
    # --------------------------------------------------------- coordinación
    def _mensaje_par(self, txt, ahora):
        try:
            m = json.loads(txt)
        except ValueError:
            return
        if not isinstance(m, dict) or m.get("q") != self.cfg.ENLACE_EQUIPO or m.get("id") == self.id:
            return
        self.par = m
        self.par_ms = ahora
        P = m.get("P")
        if P and len(P) == 3:
            self._par_pose = (float(P[0]), float(P[1]), float(P[2]), ahora)
        if "n" in m:
            self.entregas[m["id"]] = max(self.entregas.get(m["id"], 0), m["n"])
            for i, n in (m.get("N") or {}).items():
                i = int(i)
                self.entregas[i] = max(self.entregas.get(i, 0), n)

    def _anunciar(self, ahora):
        cfg = self.cfg
        # Si hace rato que no oigo al compañero (probando un robot solo, o el
        # otro apagado) anuncio 1 vez por segundo: menos ruido en la radio,
        # que es la misma que usa el WiFi de la cámara.
        periodo = cfg.ENLACE_PERIODO if ahora - self.par_ms < 5000 else 1.0
        if ahora - self._env_ms < periodo * 1000:
            return
        self._env_ms = ahora
        if self.enlace is None:
            return
        m = {"q": cfg.ENLACE_EQUIPO, "id": self.id, "e": self.estado, "t": self.tarea,
             "n": self.entregas.get(self.id, 0),
             "N": dict((str(k), v) for k, v in self.entregas.items()),
             "dm": round(self.cal.desfase) % 360}
        m["k"] = [[round(q[0], 1), round(q[1], 1)] for q in self.mi_corredor()]
        if self.pose is not None and self.est.ini:
            m["P"] = [round(self.pose[0], 1), round(self.pose[1], 1), round(self.rumbo())]
        if getattr(self, "freno", "") == "par" and self.espera_ms:
            m["f"] = 1            # "estoy frenado por vos": el otro se corre
        if self.objetivo is not None and self.estado not in ("FIN", "ESPERA"):
            m["o"] = [round(self.objetivo[0], 1), round(self.objetivo[1], 1)]
        if self.plan is not None and ahora - self.plan_ms < 3000 and self._soy_lider():
            m["p"] = dict((str(k), v) for k, v in self.plan.items())
        # ESP-NOW manda como máximo 250 bytes por mensaje: si no entra, se
        # sacan primero las partes menos importantes.
        txt = _json(m)
        for k in ("p", "o", "N", "k"):
            if len(txt) <= 240:
                break
            m.pop(k, None)
            txt = _json(m)
        try:
            self.enlace.enviar(txt)
        except Exception:  # noqa — la radio nunca puede tumbar al robot
            pass

    def _soy_lider(self):
        i = self.id_par()
        return i is None or self.id < i

    def _contar_entregas(self):
        """Cuando un cubo pasa a entregado, se le anota al rover más cercano.
        Los dos rovers ven lo mismo, así que anotan lo mismo sin hablarse."""
        for color in self.w.cubos:
            ent = self.entregado(color)
            antes = self.prev_entregado.get(color, ent)
            if ent and not antes:
                c = self.cubo(color)
                quien = None
                dmin = 1e9
                for i, r in self.w.rovers.items():
                    d = G.dist(c, (r[0], r[1]))
                    if d < dmin:
                        dmin, quien = d, i
                if quien is not None and dmin < self.cfg.AGARRE + self.cfg.RETROCESO + 6:
                    self.entregas[quien] = self.entregas.get(quien, 0) + 1
                    self.log("Cubo {} entregado (se le anota al rover {})".format(color, quien))
            self.prev_entregado[color] = ent

    def _calcular_plan(self, ahora):
        cfg = self.cfg
        pend = [c for c in self.pendientes() if self.excluir.get(c, 0) < ahora]
        if not pend:
            pend = self.pendientes()
        rovers = {}
        rovers[self.id] = self.mi_pose()
        pp = self.pose_par()
        if pp is not None:
            rovers[self.id_par()] = (pp[0], pp[1], pp[2])
        oblig = [i for i in rovers if self.entregas.get(i, 0) == 0]
        fijos = {}
        if self.tarea and self.estado in CON_CUBO:
            fijos[self.id] = self.tarea
        elif self.tarea in pend and self.estado in ("IR", "ALINEAR", "ESPERAR", "APARTAR", "ELEGIR") \
                and not self.lo_lleva_el_par(self.tarea):
            # ya iba por este cubo: lo sigo teniendo (si no, el plan cambiaba de
            # idea a cada rato al moverse los robots y ninguno iba por ninguno)
            fijos[self.id] = self.tarea
        if pp is not None and self.par_fresco() and self.par.get("e") in ("IR", "ALINEAR") \
                and self.par.get("t") in pend and self.par.get("t") != fijos.get(self.id):
            fijos[self.id_par()] = self.par.get("t")
        if pp is not None:
            for c in pend:
                if c != fijos.get(self.id) and self.lo_lleva_el_par(c):
                    fijos[self.id_par()] = c
        # Si soy seguidor y el líder mandó un plan reciente y coherente, lo uso
        if not self._soy_lider() and self.par_fresco(1500) and self.par.get("p"):
            p = {}
            for k, v in self.par["p"].items():
                p[int(k)] = [c for c in v if c in pend]
            todos = []
            for v in p.values():
                todos += v
            if sorted(todos) == sorted(pend) and self.id in p:
                choca = fijos.get(self.id) and fijos[self.id] not in p[self.id]
                if not choca:
                    return p
        # El reparto cuesta tiempo en el ESP32: si nada importante cambió, se
        # reusa el último por un segundo y medio.
        clave = (str(sorted(pend)), str(sorted(rovers.keys())), str(oblig), str(fijos))
        if clave == getattr(self, "_plan_clave", None) and ahora - getattr(self, "_plan_cache_ms", -99999) < 1500:
            return self._plan_cache
        p = P.repartir(cfg, self.w, pend, rovers, oblig, fijos)
        self._plan_clave = clave
        self._plan_cache = p
        self._plan_cache_ms = ahora
        return p

    # ----------------------------------------------------- máquina de estados
    def _cedo(self):
        return self._par_primero()

    # ------------------------------------------------------------ corredores
    # Cada rover "reserva" el camino que le falta para su tarea actual (una
    # línea quebrada: él -> detrás del cubo -> cubo -> entrada de la zona) y lo
    # anuncia por radio. Un rover no empieza una tarea cuyo camino pase a
    # menos de CORREDOR celdas del camino reservado por el otro: espera o
    # elige otro cubo. Así casi nunca se encuentran de frente.

    def _corredor_tarea(self, color, desde):
        cfg = self.cfg
        if color is None or color not in self.w.cubos:
            return [desde]
        c = self.cubo(color)
        zona = P.objetivo_zona(self.cfg, self.w, color)
        th_in = P.theta_entrada(self.w, color)
        previo_z = G.mover(zona, th_in + 180.0, cfg.AGARRE + cfg.APROX_ZONA)
        final = G.mover(zona, th_in + 180.0, cfg.AGARRE)
        e = self.estado
        if e in ("LLEVAR", "SOLTAR") and color == self.tarea:
            return [desde, previo_z, final]
        if e == "CAPTURAR" and color == self.tarea:
            return [desde, c, previo_z, final]
        pre, _ = P.punto_previo(cfg, self.w, color, [])
        return [desde, pre, c, previo_z, final]

    def mi_corredor(self):
        p = (self.pose[0], self.pose[1]) if self.pose else (0.0, 0.0)
        if self.estado in ACTIVOS and self.tarea:
            if self.estado == "SOLTAR":
                return [p]
            k = self._corredor_tarea(self.tarea, p)
            # si la ruta real rodea obstáculos, el corredor sigue la ruta real
            # (si no, el compañero cree que paso por un lado y paso por otro)
            if self.ruta and self.estado in ("IR", "LLEVAR") and len(self.ruta) > 1 \
                    and self._ahora - self.ruta_ms < 1500:
                k = [p] + [tuple(q) for q in self.ruta[:3]] + k[2:]
            return k
        return [p]

    def corredor_par(self):
        pp = self.pose_par()
        if pp is None:
            return []
        a = (pp[0], pp[1])
        if self.par_fresco() and self.par.get("k"):
            k = [tuple(q) for q in self.par["k"]]
            k[0] = a                       # el primer punto es donde está AHORA
            return k
        if self._rango_par() > 0:
            return [a, G.mover(a, pp[2], 8.0)]
        return [a]

    def _dist_corredores(self, k1, k2):
        if not k1 or not k2:
            return 99.0
        segs1 = [(k1[i], k1[i + 1]) for i in range(len(k1) - 1)] or [(k1[0], k1[0])]
        segs2 = [(k2[i], k2[i + 1]) for i in range(len(k2) - 1)] or [(k2[0], k2[0])]
        d = 99.0
        for a in segs1:
            for b in segs2:
                x = G.dist_seg_seg(a[0], a[1], b[0], b[1])
                if x < d:
                    d = x
        return d

    def _rango(self, e):
        return RANGO.get(e, 0)

    def _rango_par(self):
        if self.par_fresco():
            return self._rango(self.par.get("e"))
        pp = self.pose_par()
        if pp is None:
            return 0
        if any(self.cubo_en_paletas((pp[0], pp[1], pp[2]), c, 0.5) for c in self.w.cubos):
            return 3
        vp = self._vp
        return 1 if (vp[0] * vp[0] + vp[1] * vp[1]) > 1.0 else 0

    def _seguir_par(self, ahora):
        """Guarda dónde estuvo el compañero (una muestra por segundo)."""
        pp = self.pose_par()
        h = getattr(self, "_par_hist", None)
        if h is None:
            h = self._par_hist = []
        if pp is None:
            return
        if not h or ahora - h[-1][0] >= 1000:
            h.append((ahora, pp[0], pp[1]))
            if len(h) > 12:
                h.pop(0)

    def _par_atascado(self):
        """¿El compañero lleva ~8 s sin moverse? (trabado, sin WiFi, apagado...)
        Entonces no le cedo el paso: si no, me quedo esperando para siempre
        (pasó en casa: el 11 nunca arrancó porque el 10 se había quedado sin WiFi)."""
        h = getattr(self, "_par_hist", None)
        if not h or len(h) < 9 or self._ahora - h[-9][0] > 10000:
            return False
        a = h[-9]
        if not all(G.dist((a[1], a[2]), (q[1], q[2])) < 1.5 for q in h[-8:]):
            return False
        # solo si además no se oye por radio (sin WiFi, colgado o apagado): si
        # habla, sabe lo que hace y le hago caso
        return not self.par_fresco(2000)

    def _esperar_ir(self, ahora, e):
        if e in ("CAPTURAR", "LLEVAR", "SOLTAR", "VERIFICAR", "LIBRE", "FIN"):
            self._esperando_desde = None
        if e not in ("IR", "ALINEAR") or not self._par_primero() or \
                self._dist_corredores(self.mi_corredor(), self.corredor_par()) >= self.cfg.CORREDOR:
            return False
        # Un solo reloj de espera para ELEGIR e IR (antes eran dos que se
        # reiniciaban entre si y el robot esperaba 20-25 s): si ya espere
        # ESPERA_MAX_PAR_MS, voy igual (el anti-choques me cuida).
        ini = getattr(self, "_esperando_desde", None)
        if ini is None:
            self._esperando_desde = ini = ahora
        return ahora - ini < getattr(self.cfg, "ESPERA_MAX_PAR_MS", 4000)

    def _ceder_al_par(self):
        """Regla 12.2.13: queda un solo cubo, yo ya entregué y el compañero
        no (y anda bien): ese cubo es suyo."""
        par = self.id_par()
        return par is not None and self.pose_par() is not None and len(self.pendientes()) <= 1 \
            and self.entregas.get(self.id, 0) > 0 \
            and self.entregas.get(par, 0) == 0 and self.par_fresco(3000) and not self._par_atascado()

    def _par_primero(self, mi_rango=None):
        """¿El compañero tiene preferencia sobre mí?"""
        if self.pose_par() is None or self._par_atascado():
            return False
        a = self._rango(self.estado) if mi_rango is None else mi_rango
        b = self._rango_par()
        if b != a:
            return b > a
        if b == 0:
            return False
        return self.id_par() < self.id

    def _estorbo(self, pose):
        """¿Estoy parado dentro del camino reservado del compañero?"""
        if self.pose_par() is None or not self._par_primero():
            return False
        if self.par_fresco(600) and self.par.get("f") and self.par.get("e") in ACTIVOS:
            return True           # me avisó por radio que lo estoy trabando
        k = self.corredor_par()
        if len(k) < 2:
            return False
        a0, a1 = G.capsula(pose, self.cfg.COLA, self.cfg.PUNTA)
        k = list(k)
        largo = G.dist(k[0], k[1])
        if largo > 6.0:
            k[0] = G.mover(k[0], G.rumbo(k[0], k[1]), 6.0)
        else:
            k = k[1:]
        return self._dist_corredores([a0, a1], k) < self.cfg.CORREDOR - 2.0

    def _apartar(self, ahora, pose):
        """Me corro hacia el costado de la línea por donde viene el compañero."""
        cfg = self.cfg
        pp = self.pose_par()
        if pp is None or self._t(ahora) > 6.0:
            self.cambiar("ESPERAR", ahora, "listo")
            return 0.0, 0.0
        if not hasattr(self, "_apartar_a") or self._t(ahora) < 0.05:
            k = self.corredor_par()
            m = cfg.MARGEN_BORDE
            best = None
            for ang in range(0, 360, 30):
                for dist in (6.0, 10.0, 14.0):
                    q = G.mover((pose[0], pose[1]), ang, dist)
                    if not (m <= q[0] <= self.w.cols - m and m <= q[1] <= self.w.rows - m):
                        continue
                    lejos = self._dist_corredores([q], k)
                    pen = sum(1 for c in self.w.cubos.values() if G.dist(q, (c[0], c[1])) < 6
                              or G.dist_punto_segmento((c[0], c[1]), (pose[0], pose[1]), q) < 4.5)
                    giro = abs(G.dif_ang(ang, pose[2]))
                    giro = min(giro, 180.0 - giro)       # adelante o marcha atrás, lo que gire menos
                    dq = G.dist(q, (pp[0], pp[1]))
                    cerca = min(dq, 16.0)                       # lejos de donde está él AHORA
                    acerca = max(0.0, G.dist(pose, (pp[0], pp[1])) - dq)   # no ir hacia él
                    pt = min(lejos, cfg.CORREDOR + 2) - 10 * pen - 0.15 * dist - 0.06 * giro \
                        + 0.4 * cerca - 1.0 * acerca
                    if best is None or pt > best[0]:
                        best = (pt, q)
            if best is None:
                best = (0, (pose[0], pose[1]))
            self._apartar_a = best[1]
        self.objetivo = self._apartar_a
        if G.dist(pose, self._apartar_a) < 1.5 or (self._t(ahora) > 1.0 and not self._estorbo(pose)):
            self.cambiar("ESPERAR", ahora, "fuera del camino")
            return 0.0, 0.0
        err = G.dif_ang(G.rumbo(pose, self._apartar_a), pose[2])
        if abs(err) > 110:
            # el lugar está detrás: voy en marcha atrás (sin dar la vuelta)
            e2 = G.dif_ang(err, 180.0)
            v, w = -cfg.VEL_CRUCERO * 0.7, G.limitar(cfg.KP_RUMBO * e2, -0.3, 0.3)
        else:
            v, w = self._ir_a(pose, self._apartar_a, cfg.VEL_CRUCERO, True)
        return self._vigilar(ahora, v, w, destrabando=True)

    def _gano_conflicto(self):
        """Si los dos van por el mismo cubo: se lo queda el más cercano
        (y si empatan, el de ID menor)."""
        c = self.cubo(self.tarea)
        pp = self.pose_par()
        if c is None or pp is None:
            return True
        # regla 12.2.13 (cada rover entrega al menos uno): gana el que no entregó
        yo, el = self.entregas.get(self.id, 0), self.entregas.get(self.id_par(), 0)
        if (yo == 0) != (el == 0):
            return yo == 0
        dm = G.dist(self.pose, c)
        dp = G.dist((pp[0], pp[1]), c)
        if abs(dm - dp) < 1.0:
            return self.id < self.id_par()
        return dm < dp
