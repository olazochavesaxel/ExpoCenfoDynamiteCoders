# movimiento.py — convertir "quiero ir a tal punto" en potencia de motores:
# girar, avanzar corrigiendo el rumbo, esquivar, frenar ante el compañero,
# no tocar cubos ya entregados y no salirse del tablero. Parte de Cerebro.

import geometria as G
import plan as P
from estados import LLEVANDO


class Movimiento:

    # ------------------------------------------------------------ obstáculos
    def _obstaculos(self, llevando):
        cfg = self.cfg
        obs = []
        extra = 1.5 if llevando else 0.0
        for color, c in self.w.cubos.items():
            if color == self.tarea:
                continue
            r = cfg.RADIO_CUBO_OBST
            borde = min(c[0], c[1], self.w.cols - c[0], self.w.rows - c[1])
            if self.entregado(color) or borde < 4.0:
                r = cfg.PUNTA + 2.0          # a estos no hay que tocarlos ni con la punta
            obs.append(((c[0], c[1]), r + extra))
        pp = self.pose_par()
        if pp is not None:
            # si hace rato que el compañero me tapa el paso, lo rodeo más de lejos
            mas = 4.0 if self._ahora < getattr(self, "_par_grande_hasta", 0) else 0.0
            obs.append(((pp[0], pp[1]), cfg.RADIO_ROVER_OBST + mas))
            obs.append((G.mover((pp[0], pp[1]), pp[2], cfg.PUNTA - 2.0), cfg.RADIO_ROVER_OBST - 1.0 + mas))
        return obs

    def _seguir_ruta(self, ahora, pose, destino, vmax, obs, final=True, arco=False):
        """Recalcula la ruta cada medio segundo y avanza al próximo punto."""
        if ahora - self.ruta_ms > 800 or not self.ruta:
            self.ruta = P.ruta(self.cfg, self.w, (pose[0], pose[1]), destino, obs)
            self.ruta_ms = ahora
        while len(self.ruta) > 1 and G.dist(pose, self.ruta[0]) < 2.0:
            self.ruta.pop(0)
        wp = self.ruta[0]
        self.objetivo = wp
        es_final = len(self.ruta) == 1 and final
        return self._ir_a(pose, wp, vmax, es_final, None, arco)

    # ---------------------------------------------------------------- tareas
    # --------------------------------------------------------------- control
    def _girar_a(self, th, wmax):
        cfg = self.cfg
        err = G.dif_ang(th, self.rumbo())
        if abs(err) < cfg.TOL_GIRO and abs(self.est.vel) < 30:
            return 0.0, 0.0, True
        return 0.0, self._w_giro(err, wmax), False

    def _w_giro(self, err, wmax):
        """Giro en el lugar con anticipación: si ya viene girando rápido hacia
        el objetivo, afloja antes para no pasarse (menos zigzag)."""
        cfg = self.cfg
        fz = getattr(self, "_giro_forzado", None)
        if fz is not None:
            if self._ahora > fz[1] or abs(err) < 25.0:
                self._giro_forzado = None
            elif (err > 0) != (fz[0] > 0):
                err = err - 360.0 if err > 0 else err + 360.0     # la vuelta larga
        self._giro_falta = (abs(err), self._ahora)    # para no prever más giro del que falta
        if self.est.con_giro:
            err = err - self.est.vel * getattr(cfg, "ANTICIPO_GIRO", 0.12)
        return G.limitar(cfg.KP_GIRO * err, -wmax, wmax)

    def _ir_a(self, pose, p, vmax, final, wmax=None, arco=False):
        cfg = self.cfg
        if wmax is None:
            wmax = cfg.GIRO_MAX
        d = G.dist(pose, p)
        self.objetivo = p
        if d < cfg.TOL_PUNTO and final:
            return 0.0, 0.0
        err = G.dif_ang(G.rumbo(pose, p), pose[2])
        if abs(err) > cfg.GIRAR_EN_LUGAR:
            if arco:
                # con el cubo entre las paletas no se gira en el lugar (el cubo
                # se sale por el costado): curva (con más fuerza, ver _motores)
                return vmax * 0.45, self._w_giro(err, wmax)
            return 0.0, self._w_giro(err, wmax)
        v = vmax
        if final:
            v = vmax * G.limitar(d / cfg.FRENADO, 0.45, 1.0)
        v *= 1.0 - abs(err) / 90.0
        w = G.limitar(cfg.KP_RUMBO * err, -0.35, 0.35)
        if arco:
            # con el cubo, las DOS ruedas hacia adelante (si una queda quieta,
            # con el peso del cubo el robot no avanza ni gira)
            w = G.limitar(w, -0.7 * v, 0.7 * v)
        return v, w

    def _vigilar(self, ahora, v, w, destrabando=False):
        """Anti-choques con el compañero y con el borde, y detector de trabas."""
        cfg = self.cfg
        pose = self.mi_pose()
        frenado = False
        self.freno = ""
        # PARE Y SIGA: con la cámara lenta (en casa ~1,6 fotos/s, 0,65 s de
        # atraso y el marcador borroso cuando el robot anda) el robot no puede
        # andar mucho a ciegas: en las pruebas se iba hasta el borde creyendo
        # que estaba en otro lado. Anda un "tramo" corto, se para, y no sigue
        # hasta que llega una foto sacada DESPUÉS de pararse (quieto, el
        # marcador sale nítido y la posición es exacta). Cerca de la zona los
        # tramos son más cortos. Girar en el lugar gasta menos tramo: el
        # giroscopio sabe cuánto giró.
        if not destrabando and (abs(v) > 0.02 or abs(w) > 0.02):
            parada = getattr(self, "_parada_ms", None)
            if parada is not None:
                if getattr(self, "_foto_t", -1) >= parada + cfg.MARGEN_FOTO_MS:
                    self._parada_ms = None          # ya me vi quieto: sigo
                    self._tramo = 0.0
                    self._tramo_ini = (self.est.dx, self.est.dy)
                    if self._revisar_tramo(ahora):
                        return 0.0, 0.0
                else:
                    self.freno = "espero-foto"
                    self.mov_ref = None
                    self.giro_ref = None
                    self.hw.led((60, 30, 0))
                    return 0.0, 0.0
            dt = getattr(self, "dt_medio", 0.03)
            if abs(v) > 0.05:
                self._tramo_avanzo = True
            if getattr(self, "_tramo_cam", None) is None and self.pose_cam is not None:
                self._tramo_cam = self.pose_cam
            self._tramo = getattr(self, "_tramo", 0.0) + dt
            ini = getattr(self, "_tramo_ini", None)
            if ini is None:
                ini = self._tramo_ini = (self.est.dx, self.est.dy)
            fr = getattr(self, "_fotos_rec", [])
            rapida = len(fr) >= 5 and self._ahora - fr[-5] < 1000 and self._ahora - fr[-1] < 250 \
                and self.est.latencia < cfg.FOTO_FRESCA_MS
            if rapida:
                # la cámara me está viendo AHORA (cámara rápida): no hace falta parar
                self._tramo = 0.0
                ini = self._tramo_ini = (self.est.dx, self.est.dy)
            anduve = G.dist(ini, (self.est.dx, self.est.dy))
            if abs(v) <= 0.02:
                tope_d, tope_t = 99.0, cfg.TRAMO_GIRO_S     # girando en el lugar
            elif self._cerca_de_zona() or (v > 0 and self.estado in ("CAPTURAR", "LLEVAR") and self._punta_al_borde(pose)):
                # (empujando un cubo cerca del borde, tramos cortos: en casa un
                # robot que no se veía andando se salió del tablero con el cubo)
                tope_d, tope_t = cfg.TRAMO_FINO, cfg.TRAMO_MAX_S
            elif self._campo_abierto(pose):
                tope_d, tope_t = cfg.TRAMO_LARGO, cfg.TRAMO_MAX_S
            else:
                tope_d, tope_t = cfg.TRAMO, cfg.TRAMO_MAX_S
            if anduve > tope_d or self._tramo > tope_t:
                self._parada_ms = self._ahora
                self.freno = "espero-foto"
                self.mov_ref = None
                self.giro_ref = None
                return 0.0, 0.0
        pp = self.pose_par()
        if pp is not None:
            self._velocidad_par()
            cedo = self._cedo()
            d = G.dist(pose, (pp[0], pp[1]))
            rel = G.dif_ang(G.rumbo(pose, (pp[0], pp[1])), pose[2])
            ahora_h = self._holgura(pose, pp)
            futuro = self._riesgo(pose, v, w)
            quieto = self._riesgo(pose, 0.0, 0.0)
            limite = 2.0 if cedo else 1.3     # holgura mínima (celdas): la posición tiene ~1 celda de error
            if futuro < limite and futuro < ahora_h - 0.05:
                # mi movimiento me acerca demasiado al otro
                if abs(v) <= 0.02 and abs(w) > 0 and not cedo and self.estado != "LLEVAR":
                    # quería girar en el lugar: me separo primero
                    v2 = cfg.VEL_CAPTURA if abs(rel) > 80 else -cfg.VEL_CAPTURA
                    sep = None
                    for vv in (v2, -v2):        # primero alejándome de él; si no, al revés
                        if self._riesgo(pose, vv, 0.0) >= ahora_h and \
                                self._choca_cubo(pose, vv, 0.0) is None and (vv > 0 or self._cola_libre(pose)):
                            sep = vv
                            break
                    if sep is not None:
                        v, w = sep, 0.0
                        self.freno = "par-me-separo"
                    elif self._riesgo(pose, 0.0, -w, True) >= ahora_h - 0.05 and self._choca_cubo(pose, 0.0, -w) is None:
                        # girando para el otro lado no me acerco: doy la vuelta larga
                        self._giro_forzado = (-1.0 if w > 0 else 1.0, ahora + 3000)
                        w = -w
                        self.freno = "par-otro-lado"
                    else:
                        v, w, frenado = 0.0, 0.0, True
                        self.freno = "par-giro"
                elif abs(v) > 0.02 and abs(w) > 0.02 and \
                        self._riesgo(pose, v, 0.0) >= max(limite + 0.3, ahora_h - 0.05):
                    w = 0.0               # sin girar no me acerco: sigo derecho un poco
                    self.freno = "par-recto"
                else:
                    v, w, frenado = 0.0, 0.0, True
                    self.freno = "par"
            # Ya tengo la punta de las paletas pegada al otro: no avanzo más
            # (empujando al otro las ruedas patinan, el robot cree que avanzó y
            # la cuenta de arriba deja de servir).
            if v > 0.02 and not frenado and self._punta_toca(pp):
                v, frenado = 0.0, True
                self.freno = "par-punta"
            if frenado and quieto < 0.6 and cedo and not destrabando:
                self._destrabar(ahora, "el companero viene hacia mi")
                return 0.0, 0.0
        # No sacar de su zona un cubo ya entregado, ni empujar un cubo que
        # está cerca del borde (se puede caer). Con los demás cubos, la ruta
        # ya los esquiva; si igual se toca uno, no es grave: se replanifica.
        if (abs(v) > 0.02 or abs(w) > 0.02) and not frenado:
            self._cubo_freno = self._choca_cubo(pose, v, w)
            if self._cubo_freno is not None:
                if abs(v) <= 0.02 and self.estado != "LLEVAR" and (not destrabando or self.estado == "APARTAR") \
                        and ahora > getattr(self, "_adelante_hasta", 0) \
                        and self._choca_cubo(pose, -cfg.VEL_CAPTURA, 0.0) is None and self._cola_libre(pose) \
                        and self._par_ok(pose, -cfg.VEL_CAPTURA, 0.0):
                    v, w = -cfg.VEL_CAPTURA, 0.0     # retrocedo un poco para poder girar
                    self.freno = "retrocedo-por-cubo"
                elif abs(v) > 0.02 and abs(w) > 0.02 and self._choca_cubo(pose, 0.0, w) is None \
                        and self._par_ok(pose, 0.0, w):
                    v = 0.0               # avanzando en curva lo toco: giro sin avanzar
                    self.freno = "giro-sin-avanzar"
                elif abs(v) <= 0.02 and abs(w) > 0.02 and self._choca_cubo(pose, 0.0, -w) is None \
                        and self._par_ok(pose, 0.0, -w):
                    # girando para este lado lo golpeo: doy la vuelta por el otro lado
                    self._giro_forzado = (-1.0 if w > 0 else 1.0, ahora + 3000)
                    w = -w
                    self.freno = "giro-por-el-otro-lado"
                else:
                    v, w, frenado = 0.0, 0.0, True
                    self.freno = "cubo"
                    self.ruta_ms = -100000
        # no sacar las paletas fuera del tablero
        if v > 0.02:
            punta = G.mover(pose, pose[2], cfg.PUNTA)
            # con un cubo adelante (agarrando o llevando) se es más estricto: el
            # cubo sobresale de las paletas y se podría caer del tablero
            lim = 0.5 if self.estado in ("CAPTURAR", "LLEVAR") else 1.5
            cerca = not (3.0 - lim < punta[0] < self.w.cols - 3.0 + lim and
                         3.0 - lim < punta[1] < self.w.rows - 3.0 + lim)
            if cerca and self.estado in ("CAPTURAR", "LLEVAR") and ahora - self.pose_ms > 900:
                v, frenado = 0.0, True          # cerca del borde y sin foto reciente: espero
                self.freno = 'borde-sin-foto'
            elif not (-lim < punta[0] < self.w.cols + lim and -lim < punta[1] < self.w.rows + lim):
                v, frenado = 0.0, True
                self.freno = 'borde-punta'
        elif v < -0.02:
            cola = G.mover(pose, pose[2], -cfg.COLA - 1.0)
            if not (0.5 < cola[0] < self.w.cols - 0.5 and 0.5 < cola[1] < self.w.rows - 0.5):
                v, frenado = 0.0, True
                self.freno = 'borde-cola'
            elif ahora - self.pose_ms > 1600:
                # sin foto reciente NO retrocedo: en la U la conexión con la
                # cámara se cortaba y los robots retrocedían a ciegas hasta caerse
                v, frenado = 0.0, True
                self.freno = 'atras-sin-foto'
        if frenado:
            self.hw.led((80, 40, 0))
            if self.espera_ms == 0:
                self.espera_ms = ahora
            elif (ahora - self.espera_ms) / 1000.0 > cfg.ESPERA_MAX and not destrabando:
                self._destrabar(ahora, "esperando demasiado")
                return 0.0, 0.0
        else:
            self.espera_ms = 0
            self.hw.led((0, 60, 0) if self.estado not in LLEVANDO else (0, 60, 60))
        # ¿trabado? mando avanzar pero no me muevo
        if abs(v) > 0.1 and not destrabando:
            if self.mov_ref is None:
                self.mov_ref = (ahora, pose[0], pose[1])
            elif ahora - self.mov_ref[0] > 2500:
                if G.dist(pose, (self.mov_ref[1], self.mov_ref[2])) < 0.8:
                    self._destrabar(ahora, "trabado")
                    return 0.0, 0.0
                self.mov_ref = (ahora, pose[0], pose[1])
        else:
            self.mov_ref = None
        # ¿trabado girando? mando girar en el lugar pero el rumbo no cambia
        if abs(v) <= 0.02 and abs(w) > 0.12 and not destrabando:
            if getattr(self, "giro_ref", None) is None:
                self.giro_ref = (ahora, pose[2])
            elif ahora - self.giro_ref[0] > 2500:
                if abs(G.dif_ang(pose[2], self.giro_ref[1])) < 8.0:
                    self.giro_ref = None
                    self._destrabar(ahora, "no puedo girar")
                    return 0.0, 0.0
                self.giro_ref = (ahora, pose[2])
        else:
            self.giro_ref = None
        return v, w

    def _velocidades(self, v, w):
        """(celdas/s, grados/s) reales que produciría el comando (v, w), con lo
        que el robot fue midiendo de sí mismo (odometría y giroscopio)."""
        cfg = self.cfg
        m = cfg.MOTOR_MINIMO
        u = 0.0
        if abs(v) > 0.02:
            u = max(0.0, m + (1.0 - m) * abs(v) - 0.7 * m) * (1.0 if v > 0 else -1.0)
        # para prever choques se calcula PESIMISTA (zona muerta chica y la
        # escala más alta): mejor frenar de más que creer que uno avanza menos
        # de lo que de verdad avanza
        vel = max(self.est.escalas) * u
        gk = getattr(self, "_giro_k", None)
        if gk is None:
            gk = cfg.GRADOS_POR_S / max(0.1, cfg.GIRO_MAX)
        return vel, w * gk

    def _medir_giro(self):
        """Aprende cuántos grados/s gira por unidad de w (girando en el lugar)."""
        v, w = self.cmd
        if abs(v) < 0.02 and abs(w) > 0.1 and self.est.con_giro and abs(self.est.vel) > 10:
            k = abs(self.est.vel) / abs(w)
            gk = getattr(self, "_giro_k", None)
            self._giro_k = k if gk is None else 0.97 * gk + 0.03 * k

    def _par_ok(self, pose, v, w):
        """¿Moverme así me deja a buena distancia del compañero?"""
        pp = self.pose_par()
        if pp is None:
            return True
        r = self._riesgo(pose, v, w, True)
        return r >= 1.0 or r >= self._holgura(pose, pp) - 0.05

    def _cola_libre(self, pose):
        cola = G.mover(pose, pose[2], -self.cfg.COLA - 2.0)
        return 0.5 < cola[0] < self.w.cols - 0.5 and 0.5 < cola[1] < self.w.rows - 0.5

    def _penetra(self, pose, q, tarea):
        """Cuánto se mete el cubo q en el robot (celdas; <=0 = no lo toca).
        Para el cubo de la tarea, el hueco entre las paletas está permitido."""
        cfg = self.cfg
        a, i = G.a_marco_robot(pose, q)
        r = 1.6                                       # medio cubo + margen
        frente = cfg.AGARRE - 1.5                     # dónde empieza el hueco
        hueco = cfg.MEDIO_ANCHO - 0.6                 # medio ancho del hueco
        # cuerpo
        pen = min(a - (-cfg.COLA - r), (frente + r) - a, (cfg.MEDIO_ANCHO + r) - abs(i))
        if tarea:
            # paletas: dos franjas delgadas a los costados del hueco
            pen2 = min(a - frente, (cfg.PUNTA + r) - a, (cfg.MEDIO_ANCHO + r) - abs(i),
                       abs(i) - (hueco - r))
        else:
            pen2 = min(a - frente, (cfg.PUNTA + r) - a, (cfg.MEDIO_ANCHO + r) - abs(i))
        return max(pen, pen2)

    def _revisar_tramo(self, ahora):
        """Al terminar un tramo, ¿me moví de verdad? (lo dice la foto, no la
        cuenta del robot). Si mandé avanzar y no me moví, los motores no
        alcanzan (batería baja, una rueda trabada): se sube un poco la
        potencia mínima. Si sigue sin moverse, retrocede y avisa."""
        ini = getattr(self, "_tramo_cam", None)
        avanzo = getattr(self, "_tramo_avanzo", False)
        pc = self.pose_cam
        self._tramo_cam = pc
        self._tramo_avanzo = False
        if ini is None or pc is None or not avanzo:
            return False
        real = G.dist(ini, pc)
        giro = abs(G.dif_ang(pc[2], ini[2]))
        extra = getattr(self, "_extra", 0.0)
        if real < 0.5 and giro < 8.0:
            self._sin_mover = getattr(self, "_sin_mover", 0) + 1
            self._extra = min(0.25, extra + 0.05)
            if self._sin_mover >= 4:
                self._sin_mover = 0
                self._destrabar(ahora, "no me muevo aunque mando potencia (bateria baja?)")
                return True
        else:
            self._sin_mover = 0
            self._extra = max(0.0, extra - 0.01)
        return False

    def _campo_abierto(self, pose):
        """Lejos del borde, del compañero y de los cubos que no llevo."""
        if self.estado not in ("IR", "LLEVAR"):
            return False
        if min(pose[0], pose[1], self.w.cols - pose[0], self.w.rows - pose[1]) < 9.0:
            return False
        pp = self.pose_par()
        if pp is not None and G.dist(pose, (pp[0], pp[1])) < 16.0:
            return False
        for k, c in self.w.cubos.items():
            if k != self.tarea and G.dist(pose, (c[0], c[1])) < 9.0:
                return False
        if self.tarea is not None and self.estado == "IR":
            c = self.cubo(self.tarea)
            if c is not None and G.dist(pose, c) < 12.0:
                return False
        return True

    def _punta_al_borde(self, pose):
        p = G.mover(pose, pose[2], self.cfg.PUNTA)
        return min(p[0], p[1], self.w.cols - p[0], self.w.rows - p[1]) < 5.0

    def _cerca_de_zona(self):
        """¿Estoy empujando (o por agarrar) un cubo que ya está cerca de su zona?"""
        if self.estado not in ("CAPTURAR", "LLEVAR") or self.tarea is None:
            return False
        z = self.w.zonas.get(self.tarea)
        if z is None:
            return False
        c = G.mover(self.mi_pose(), self.rumbo(), self.cfg.AGARRE) if self.estado == "LLEVAR" \
            else self.cubo(self.tarea)
        return c is not None and G.dist(c, z) < 9.0

    def _punta_toca(self, pp):
        """¿La punta de mis paletas está a menos de media celda del otro robot?
        Se mira con la pose estimada y con la de la última foto."""
        cfg = self.cfg
        poses = [self.mi_pose()]
        if self.pose_cam is not None and self._ahora - self.pose_ms < 1200:
            poses.append(self.pose_cam)
        for q in poses:
            for lado in (-cfg.MEDIO_ANCHO_PAR, 0.0, cfg.MEDIO_ANCHO_PAR):
                pt = G.a_mundo(q, cfg.PUNTA + 0.3, lado)
                if G._dist_a_rect(pp, pt, cfg.COLA, cfg.PUNTA, cfg.MEDIO_ANCHO_PAR) < 0.5:
                    return True
        return False

    def _choca_cubo(self, pose, v, w):
        """Color del cubo que golpearía con (v, w), o None."""
        cfg = self.cfg
        vel, giro = self._velocidades(v, w)
        for color, c in self.w.cubos.items():
            tarea = color == self.tarea
            if tarea:
                continue
            borde = min(c[0], c[1], self.w.cols - c[0], self.w.rows - c[1])
            protegido = self.entregado(color) or borde < (2.5 if self.estado == "LLEVAR" else 4.0)
            if not protegido and (abs(v) > 0.02 or self.estado == "LLEVAR"):
                continue    # avanzando: a los cubos comunes ya los esquiva la ruta
            q = (c[0], c[1])
            if G.dist(pose, q) > cfg.PUNTA + 6:
                continue
            ahora_p = self._penetra(pose, q, tarea)
            if protegido and ahora_p > 0.0 and abs(w) > 0.02 and G.a_marco_robot(pose, q)[0] < cfg.PUNTA + 0.8:
                return color    # ya lo toco: girando, las paletas lo sacan de su zona
            for T in (0.25, 0.5):
                th = pose[2] + giro * T
                paso = vel * T
                if paso > 0 and self.objetivo is not None and self.estado in ("IR", "APARTAR", "LIBRE"):
                    # no voy más allá de donde voy (antes frenaba ante un cubo
                    # entregado a 12 celdas cuando le faltaban 2 para llegar)
                    paso = min(paso, G.dist(pose, self.objetivo) + 1.0)
                p = G.mover((pose[0], pose[1]), pose[2], paso)
                pn = self._penetra((p[0], p[1], th), q, tarea)
                if pn > 0.0 and pn > ahora_p + 0.02:
                    return color
        return None

    def _destrabar(self, ahora, motivo):
        self._volver_a = "IR" if self.estado in ("IR", "ALINEAR", "CAPTURAR") else \
            ("LLEVAR" if self.estado == "LLEVAR" else "ELEGIR")
        if self.estado == "LLEVAR":
            self._volver_a = "IR"   # retrocedimos: hay que volver a agarrarlo
        x = getattr(self, "_cubo_freno", None)
        par = self.id_par()
        dejo = par is None or self.entregas.get(par, 0) > 0 or len(self.pendientes()) > 2
        if getattr(self, "freno", "") == "cubo" and x is not None and x != self.tarea and self.prueba is None and dejo \
                and x in self.pendientes() and not self.lo_lleva_el_par(x):
            # me bloquea un cubo que TAMBIÉN hay que llevar (por ejemplo, quedó
            # metido en otra zona): llevo ese primero y libero el paso
            self.excluir[self.tarea] = ahora + 4000
            self.tarea = x
            self._volver_a = "IR"
            self.cambiar("DESTRABAR", ahora, motivo + "; primero llevo el " + x)
            return
        if getattr(self, "freno", "") == "cubo" and self.estado in ("ALINEAR", "CAPTURAR") and self.tarea:
            # agarrarlo desde este lado choca otro cubo: la próxima vez, desde otro lado
            m = self._u_malos.get(self.tarea, [])
            self._u_malos[self.tarea] = (m + [self.mi_pose()[2]])[-3:]
        # ¿ya me trabé varias veces con ESTE cubo? lo dejo un rato y voy por otro
        # (por ejemplo, el compañero quedó parado justo al lado)
        if self.tarea and self.prueba is None:
            h = [t for t in getattr(self, "_trabas", {}).get(self.tarea, []) if ahora - t < 45000] + [ahora]
            tr = getattr(self, "_trabas", {})
            tr[self.tarea] = h
            self._trabas = tr
            if len(h) >= 3 and len(self.pendientes()) > 1:
                tr[self.tarea] = []
                self.excluir[self.tarea] = ahora + 20000
                self._volver_a = "ELEGIR"
                self.cambiar("DESTRABAR", ahora, motivo + "; pruebo con otro cubo")
                return
        self.cambiar("DESTRABAR", ahora, motivo)

    def _rampa(self, v, w):
        """Arranques suaves: acelerar y empezar a girar de a poco (ACEL_*,
        unidades de potencia por segundo). Frenar es inmediato: para no chocar,
        frenar nunca se demora."""
        cfg = self.cfg
        ahora = getattr(self, "_ahora", 0)
        dt = 0.03 if self._rampa_ms is None else (ahora - self._rampa_ms) / 1000.0
        self._rampa_ms = ahora
        dt = G.limitar(dt, 0.0, 0.1)

        def paso(actual, pedido, acel):
            if pedido == 0.0 or (actual != 0.0 and (pedido > 0) != (actual > 0)):
                return pedido if pedido == 0.0 else 0.0     # frenar / cambiar de sentido: primero a 0
            if abs(pedido) <= abs(actual):
                return pedido                                # bajar: inmediato
            s = 1.0 if pedido > 0 else -1.0
            return s * min(abs(pedido), abs(actual) + acel * dt)
        self._v_rampa = paso(self._v_rampa, v, getattr(cfg, "ACEL_AVANCE", 1.5))
        self._w_rampa = paso(self._w_rampa, w, getattr(cfg, "ACEL_GIRO", 2.5))
        return self._v_rampa, self._w_rampa

    def _motores(self, v, w):
        """v = avance, w = giro (+ antihorario). Mezcla y compensa la fricción."""
        cfg = self.cfg
        lento = getattr(self, "dt_medio", 0.03)
        if lento > 0.05:
            f = max(0.75, 0.05 / lento)
            v *= f
            w *= max(0.75, f)
        v, w = self._rampa(v, w)
        self.cmd = (v, w)
        izq = v - w
        der = v + w
        m = max(abs(izq), abs(der), 1.0)
        izq /= m
        der /= m

        def comp(x):
            if abs(x) < 0.02:
                return 0.0
            s = 1.0 if x > 0 else -1.0
            m = cfg.MOTOR_MINIMO + getattr(self, "_extra", 0.0)     # + refuerzo si no se mueve
            if self.estado == "LLEVAR":
                m += getattr(cfg, "EXTRA_CON_CUBO", 0.08)    # empujando el cubo hace falta más fuerza
            return s * (m + (1.0 - m) * abs(x))
        ci, cd = comp(izq), comp(der)
        self.hw.motores(ci, cd)
        # para la odometría: cuánto de esa potencia realmente mueve el robot
        t0 = cfg.MOTOR_MINIMO * 0.7

        def util(x):
            if x > t0:
                return x - t0
            if x < -t0:
                return x + t0
            return 0.0
        self._util = (util(ci) + util(cd)) / 2.0

    # ---------------------------------------------------- modo pruebas (panel)
