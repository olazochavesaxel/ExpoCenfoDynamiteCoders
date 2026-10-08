# arranque.py — dos ayudas de la máquina de estados (parte de Cerebro; van
# aparte para que cada archivo sea chico y el ESP32 lo compile con poca memoria):
#   - CALIBRAR: al arrancar, avanzar recto unas celdas para comprobar que el
#     marcador ArUco apunta hacia las paletas (y corregirlo si no).
#   - si sin querer se mete OTRO cubo pendiente entre las paletas, llevar ese.
#   - LLEVAR: empujar el cubo agarrado hasta su zona.

import geometria as G
import plan as P


class Arranque:

    def _rel_foto(self, color):
        """(adelante, izquierda) del cubo respecto del robot en la ÚLTIMA FOTO
        NUEVA en que se ven bien los dos; None si no hay foto nueva o alguno
        estaba tapado. Cada foto se usa una sola vez por color."""
        c = self.w.cubos.get(color)
        pc = self.pose_cam
        if c is None or pc is None or c[2] > 350 or self._edad_rover > 350:
            return None
        k = getattr(self, "_rel_vista", {})
        if k.get(color) == self._foto_n:
            return None
        if getattr(self, "_foto_t", 0) < self.estado_ms:
            return None          # foto sacada ANTES de empezar este paso (p. ej. antes de retroceder)
        k[color] = self._foto_n
        self._rel_vista = k
        return G.a_marco_robot(pc, (c[0], c[1]))

    def _calibrar(self, ahora, pose):
        """Al arrancar: avanza recto unas celdas para que el CalibradorMarcador
        compruebe que el marcador apunta hacia las paletas (y lo corrija si no).
        Se usa el giroscopio para ir recto, no la cámara, porque justamente el
        rumbo de la cámara es lo que se está comprobando."""
        cfg = self.cfg
        t = self._t(ahora)
        borde = min(pose[0], pose[1], self.w.cols - pose[0], self.w.rows - pose[1])
        if self._cal_ini is None:
            self._cal_ini = (pose[0], pose[1], borde, self.est.hg, self.cal.correcciones)
        c0, r0, b0, g0, n0 = self._cal_ini
        fin = None
        if self.cal.confirmado or self.cal.correcciones > n0:
            fin = "marcador comprobado, desfase {:.0f}".format(self.cal.desfase)
        elif t > 3.0 or G.dist((c0, r0), pose) > 7.0:
            fin = "no pude comprobar el marcador (sigo, se corrige andando)"
        elif borde < b0 - 1.0:
            fin = "iba hacia el borde"
        else:
            pp = self.pose_par()
            if pp is not None and G.dist(pose, (pp[0], pp[1])) < 6.5:
                fin = "el companero esta muy cerca"
        if fin:
            self._cal_ini = None
            self.cambiar(getattr(self, "_cal_volver", None) or "ELEGIR", ahora, fin)
            return 0.0, 0.0
        w = 0.0
        if self.est.con_giro:
            w = G.limitar(cfg.KP_RUMBO * G.dif_ang(g0, self.est.hg), -0.2, 0.2)
        self.hw.led((60, 0, 60))
        return cfg.VEL_CRUCERO * 0.8, w

    def _cubo_ajeno_en_paletas(self, pose):
        """¿Se me metió entre las paletas OTRO cubo pendiente (no el de mi tarea)?"""
        if self.prueba is not None:
            return None        # en una prueba del panel se lleva SOLO el cubo pedido
        # regla 12.2.13: cada rover tiene que entregar al menos uno. Si el
        # compañero todavía no entregó, no le saco el último que le queda.
        par = self.id_par()
        if par is not None and self.entregas.get(par, 0) == 0 and len(self.pendientes()) <= 2:
            return None
        for color in self.pendientes():
            if color == self.tarea or self.w.cubos[color][2] > self.cfg.EDAD_CUBO_FRESCO_MS \
                    or self.excluir.get(color, 0) > self._ahora:
                continue
            if self.cubo_en_paletas(self.pose_cam or pose, color, 0.3):
                pt = self.par.get("t") if self.par_fresco() else None
                if pt != color:
                    return color
        return None

    def _vigilar_avance(self, ahora, pose):
        """Si en IR o LLEVAR no me acerco a la meta en varios segundos (por
        ejemplo, el compañero quieto me tapa el camino y quedo yendo y
        viniendo), rodeo al compañero más de lejos y recalculo."""
        e = self.estado
        if e not in ("IR", "LLEVAR") or self.tarea is None:
            self._avance = None
            return
        c = self.cubo(self.tarea)
        z = self.w.zonas.get(self.tarea)
        if c is None or z is None:
            return
        d = G.dist(pose, c) if e == "IR" else G.dist(c, z)
        a = getattr(self, "_avance", None)
        if a is None or a[2] != e or d < a[1] - 1.5:
            self._avance = (ahora, d, e)
            return
        if ahora - a[0] > 7000:
            self._avance = (ahora, d, e)
            pp = self.pose_par()
            if pp is not None and G.dist(pose, (pp[0], pp[1])) < 18.0:
                self._par_grande_hasta = ahora + 12000
            self.ruta_ms = -100000
            self.log("sin avance hacia {}: recalculo el camino".format(self.tarea))

    def _llevar(self, ahora, pose):
        cfg = self.cfg
        cw = self.w.cubos[self.tarea]
        zona = P.objetivo_zona(cfg, self.w, self.tarea)
        zona_real = self.w.zonas[self.tarea]
        th_in = P.theta_entrada(self.w, self.tarea)
        if getattr(self, "_llev_key", None) != self.estado_ms:
            self._llev_key = self.estado_ms
            self._llev_fuera = 0
            self._off_lat = 0.0     # cuánto va corrido el cubo hacia la izquierda de las paletas
            self._llev_th = None
        # rumbo de entrada: derecho al borde, salvo que otro cubo tape ese
        # carril (lo arrastraría adentro o contra la pared): entro en diagonal
        if getattr(self, "_tramo_final", None) != self.estado_ms:
            th_e = self._llev_th
            if th_e is None or not self._carril_libre(zona, th_e):
                th_e = th_in
                for d in (0.0, 25.0, -25.0, 40.0, -40.0):
                    if self._carril_libre(zona, th_in + d):
                        th_e = th_in + d
                        break
                self._llev_th = th_e
        if self._llev_th is not None:
            th_in = self._llev_th
        # ¿sigue entre las paletas? Se mira en cada foto nueva (robot y cubo del
        # mismo instante); hacen falta DOS fotos seguidas para darlo por perdido.
        # Si la cámara no lo ve (lo tapan las paletas), se supone que sigue ahí.
        rel = self._rel_foto(self.tarea)
        if rel is not None:
            ok = 1.5 <= rel[0] <= cfg.AGARRE + 3.0 and abs(rel[1]) < 2.8
            self._llev_fuera = 0 if ok else self._llev_fuera + 1
            if ok:
                self._off_lat = 0.5 * self._off_lat + 0.5 * G.limitar(rel[1], -2.0, 2.0)
            if self._llev_fuera >= 2:
                self.cambiar("IR", ahora, "se me salio el cubo")
                return 0.0, 0.0
        cubo = G.a_mundo(pose, cfg.AGARRE, getattr(self, "_off_lat", 0.0))   # dónde está AHORA (va con el robot)
        lado = self.w.lado_cubo + 1.2                    # exigente: que quede bien adentro
        falta = G.falta_para_zona(cubo, zona_real, self.w.grid, self.w.zona_tam, lado)
        # el cubo puede ir corrido hacia un costado entre las paletas (la foto
        # lo dice): el robot se corre al revés para que el CUBO quede centrado
        off = getattr(self, "_off_lat", 0.0)
        zona_r = G.a_mundo((zona[0], zona[1], th_in), 0.0, -off)
        final = G.mover(zona_r, th_in + 180.0, cfg.AGARRE)             # robot aquí => cubo al centro
        # punto previo: si otro cubo queda pegado a él, la ruta no lo esquiva
        # (lo toma como "destino") y el robot lo arrastra hasta la zona. Se
        # acerca el punto previo a la zona hasta que quede libre.
        ap = cfg.APROX_ZONA
        if getattr(self, "_llev_ap", (None,))[0] == self.estado_ms:
            ap = self._llev_ap[1]
        for a2 in (ap, 3.5, 2.0):
            if a2 > ap:
                continue
            ap = a2
            pv = G.mover(zona_r, th_in + 180.0, cfg.AGARRE + a2)
            lejos = True
            for k, oc in self.w.cubos.items():
                if k != self.tarea and G.dist(pv, (oc[0], oc[1])) < (cfg.RADIO_CUBO_OBST + 1.5) * 0.8 + 0.7:
                    lejos = False
            if lejos:
                break
        self._llev_ap = (self.estado_ms, ap)
        previo = G.mover(zona_r, th_in + 180.0, cfg.AGARRE + ap)
        mas_adentro = G.mover(zona_r, th_in + 180.0, cfg.AGARRE - 0.8)  # empujar un poco más
        # Para SOLTAR no alcanza con lo que el robot CREE que avanzó: lo tiene
        # que confirmar una foto sacada mientras lo llevaba (en casa el robot
        # soltaba el cubo 8 celdas antes porque calculaba que ya había llegado).
        pc = self.pose_cam
        foto = pc is not None and ahora - self.pose_ms < 1500 and \
            getattr(self, "_foto_t", 0) >= self.estado_ms
        confirma = False
        pasado = False
        if foto:
            # dónde está el cubo en la foto: si la cámara lo ve, eso; si las
            # paletas lo tapan, delante del robot (puede estar corrido ~1 celda)
            if cw[2] < 350 and self._edad_rover < 350:
                cubo_f = (cw[0], cw[1])
            else:
                cubo_f = G.a_mundo(pc, cfg.AGARRE, getattr(self, "_off_lat", 0.0))
            confirma = G.falta_para_zona(cubo_f, zona_real, self.w.grid, self.w.zona_tam, lado) == 0.0
            # ¿el cubo ya pasó el centro de la zona (hacia el borde)? no empujar más:
            # con la cámara atrasada es fácil pasarse y sacarlo por el otro lado
            pasado = G.a_marco_robot((zona[0], zona[1], th_in), cubo_f)[0] > 0.3 and \
                G.falta_para_zona(cubo_f, zona_real, self.w.grid, self.w.zona_tam, self.w.lado_cubo + 0.6) == 0.0
            if pasado or (confirma and G.dist(pc, mas_adentro) < 1.0):
                self.cambiar("SOLTAR", ahora, "adentro (foto)")
                return 0.0, 0.0
            if G.dist(pc, mas_adentro) < 0.7:
                self.cambiar("SOLTAR", ahora, "llegue al fondo")   # VERIFICAR dice si entró
                return 0.0, 0.0
        if confirma and falta == 0.0 and (G.dist(cubo, zona) < 1.0 or G.dist(pose, final) < 0.6):
            self.cambiar("SOLTAR", ahora, "adentro")
            return 0.0, 0.0
        if (falta > 0.0 or (foto and not confirma)) and G.dist(pose, final) < 0.8:
            final = mas_adentro          # llegué y el cubo todavía no entra: empujo un poco más
        # tramo final recto hacia el borde, o primero ir al punto previo
        a_prev = G.dist(pose, previo)
        en_linea = abs(G.dif_ang(G.rumbo(pose, final), th_in)) < 20 and \
            G.dist(pose, final) <= cfg.APROX_ZONA + 1.0
        if en_linea:
            for k, oc in self.w.cubos.items():
                if k != self.tarea and G.dist_punto_segmento((oc[0], oc[1]), (pose[0], pose[1]),
                                                             G.mover(zona, th_in, 1.0)) < cfg.MEDIO_ANCHO + 1.0:
                    en_linea = False   # hay un cubo en el carril: lo rodeamos con la ruta
        if a_prev < 2.5:
            self._tramo_final = self.estado_ms      # ya estoy en el tramo final: no vuelvo atrás
        if en_linea or a_prev < 2.5 or getattr(self, "_tramo_final", None) == self.estado_ms:
            # entrando a la zona (cerca del borde): solo con una foto reciente.
            # Si la cámara hace rato que no me ve, espero quieto la próxima foto
            # en vez de seguir empujando a ciegas (el cubo se puede caer).
            v, w = self._entrar_zona(pose, final, th_in)
            # quieto en la entrada sin que ninguna foto confirme nada: no me quedo ahí para siempre
            if v == 0.0 and w == 0.0:
                q = getattr(self, "_llev_quieto", None)
                if q is None or q[0] != self.estado_ms:
                    self._llev_quieto = (self.estado_ms, ahora)
                elif ahora - q[1] > 4000:
                    self._llev_quieto = None
                    self.cambiar("SOLTAR", ahora, "no avanzo mas: suelto y miro")   # VERIFICAR decide
                    return 0.0, 0.0
            else:
                self._llev_quieto = None
        else:
            v, w = self._seguir_ruta(ahora, pose, previo, cfg.VEL_LLEVANDO,
                                     self._obstaculos(True), final=False, arco=True)
            w = G.limitar(w, -cfg.GIRO_MAX_LLEVANDO, cfg.GIRO_MAX_LLEVANDO)
        return self._vigilar(ahora, v, w)

    def _carril_libre(self, zona, th):
        cfg = self.cfg
        pv = G.mover(zona, th + 180.0, cfg.AGARRE + cfg.APROX_ZONA)
        m = cfg.MARGEN_BORDE
        if not (m <= pv[0] <= self.w.cols - m and m <= pv[1] <= self.w.rows - m):
            return False
        b = G.mover(zona, th, 1.0)
        for k, oc in self.w.cubos.items():
            if k == self.tarea:
                continue
            q = (oc[0], oc[1])
            if G.dist_punto_segmento(q, pv, b) < cfg.MEDIO_ANCHO + 2.0 or G.dist(pv, q) < 7.0:
                return False
        return True

    def _entrar_zona(self, pose, final, th_in):
        """Tramo final con el cubo: derecho hacia el borde, con el rumbo de
        entrada FIJO y corrigiendo de a poco si voy corrido hacia un costado.
        (Apuntar al punto final hace girar fuerte justo al llegar y el cubo
        sale despedido por el costado.)"""
        cfg = self.cfg
        a, lat = G.a_marco_robot((final[0], final[1], th_in), (pose[0], pose[1]))
        if a >= -0.3:
            if abs(lat) > 1.2:
                # me pasé del punto de entrada pero corrido hacia un costado (en
                # casa quedó así 25 s sin hacer nada): retrocedo y vuelvo a entrar
                return -cfg.VEL_FINAL, 0.0
            return 0.0, 0.0                       # ya llegué: espero la foto que lo confirme
        # corrido menos de 1 celda (2 cm) hacia un costado: el cubo igual entra,
        # sigo derecho (antes se quedaba corrigiendo de a poquito sin avanzar)
        lat = lat if abs(lat) > 1.0 else 0.0
        rumbo = th_in - G.limitar(lat * 6.0, -25.0, 25.0)
        err = G.dif_ang(rumbo, pose[2])
        if abs(err) < 5.0:
            err = 0.0
        w = G.limitar(cfg.KP_RUMBO * err, -cfg.GIRO_MAX_LLEVANDO, cfg.GIRO_MAX_LLEVANDO)
        if abs(err) > 35.0:
            return cfg.VEL_FINAL * 0.5, w          # muy torcido: curva lenta, sin girar en el lugar
        v = cfg.VEL_FINAL * G.limitar(-a / 3.0, 0.6, 1.0)
        return v, G.limitar(w, -0.7 * v, 0.7 * v)  # las dos ruedas hacia adelante
