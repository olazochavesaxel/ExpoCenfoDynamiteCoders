# pruebas.py — órdenes de PRUEBA que manda el panel de la compu (solo si
# config.MODO_PRUEBAS = True) y el informe de estado para el panel. Parte de Cerebro.

import geometria as G


class Pruebas:

    # ---------------------------------------------------- modo pruebas (panel)
    def _orden_prueba(self, txt, ahora):
        if not self.cfg.MODO_PRUEBAS:
            return
        p = txt.strip().split()
        if not p:
            return
        k = p[0].upper()
        self.log("Orden de prueba: " + txt.strip())
        if k == "PARAR":
            self.prueba = None
            self.auto_forzado = False
            self.reiniciar()
        elif k == "AUTO":
            self.prueba = None
            self.auto_forzado = True
            self.reiniciar()
        elif k == "CALIBRAR":
            self.prueba = {"k": k, "a": [], "ms": ahora, "n0": self.cal.mediciones, "p0": self.pose}
        elif k in ("IR", "GIRAR", "RECTO"):
            self.prueba = {"k": k, "a": [float(x) for x in p[1:]], "ms": ahora}
        elif k in ("AGARRAR", "LLEVAR") and len(p) > 1:
            self.prueba = {"k": k, "a": [], "ms": ahora}
            self.tarea = p[1].lower()
            self.estado = "IR"
            self.estado_ms = ahora
            if not self.cal.confirmado and getattr(self.cfg, "CALIBRAR_AL_ARRANCAR", False):
                self._cal_volver = "IR"
                self.cambiar("CALIBRAR", ahora, "primero compruebo el marcador")

    def _prueba(self, ahora):
        cfg = self.cfg
        if self.pose is None:
            return 0.0, 0.0
        pose = self.mi_pose()
        k = self.prueba["k"]
        a = self.prueba["a"]
        t = (ahora - self.prueba["ms"]) / 1000.0
        self.hw.led((60, 0, 60))
        if k == "IR":
            v, w = self._ir_a(pose, (a[0], a[1]), cfg.VEL_CRUCERO, True)
            if v == 0 and w == 0:
                self.prueba = None
            return v, w
        if k == "GIRAR":
            v, w, listo = self._girar_a(a[0], cfg.GIRO_MAX)
            if listo or t > 8:
                self.prueba = None
            return v, w
        if k == "CALIBRAR":
            # avanza recto hasta medir 2 tramos: así el robot mide solo el
            # desfase del marcador (ver estimador.CalibradorMarcador)
            lejos = self.prueba["p0"] is not None and G.dist(self.prueba["p0"], pose) > 8.0
            borde = min(pose[0], pose[1], self.w.cols - pose[0], self.w.rows - pose[1]) < 4.0
            if self.cal.mediciones - self.prueba["n0"] >= 2 or t > 5 or lejos or (borde and t > 0.3):
                self.prueba = None
                self.log("Calibrar: desfase del marcador = {:.0f} grados (ultimo error {})".format(
                    self.cal.desfase, self.cal.ultimo_error if self.cal.ultimo_error is None
                    else round(self.cal.ultimo_error)))
                return 0.0, 0.0
            if t < 0.05 or not hasattr(self, "_recto_th"):
                self._recto_th = pose[2]
            err = G.dif_ang(self._recto_th, pose[2])
            return cfg.VEL_CAPTURA * 1.4, G.limitar(cfg.KP_RUMBO * err, -0.2, 0.2)
        if k == "RECTO":
            dur = a[0] if a else 1.5
            vel = a[1] if len(a) > 1 else cfg.VEL_CRUCERO
            if t > dur:
                self.prueba = None
                return 0.0, 0.0
            if not hasattr(self, "_recto_th") or t < 0.05:
                self._recto_th = pose[2]
            err = G.dif_ang(self._recto_th, pose[2])
            return vel, G.limitar(cfg.KP_RUMBO * err, -0.3, 0.3)
        # AGARRAR / LLEVAR: usa la misma máquina de estados de la competencia
        if self.tarea is None or self.w.cubos.get(self.tarea) is None:
            # la cámara todavía no vio ese cubo: espero un poco antes de rendirme
            if self.tarea is not None and t < 8.0:
                return 0.0, 0.0
            if self.tarea is not None:
                self.log("La camara no ve el cubo {}: calibrar los colores".format(self.tarea))
            else:
                self.log("Prueba terminada")
            self.prueba = None
            return 0.0, 0.0
        if k == "AGARRAR" and self.estado in ("LLEVAR", "SOLTAR", "VERIFICAR"):
            self.prueba = None
            self.reiniciar()
            return 0.0, 0.0
        if self.estado in ("ELEGIR", "FIN", "LIBRE"):
            self.prueba = None
            self.reiniciar()
            return 0.0, 0.0
        return self._maquina(ahora)

    # --------------------------------------------------------------- informe
    # --------------------------------------------------------------- informe
    def informe(self):
        """Resumen para el panel de la compu (solo lectura)."""
        d = {"id": self.id, "e": self.estado, "f": getattr(self, "freno", ""), "t": self.tarea, "m": self.motivo,
             "h": round(self.rumbo(), 1), "n": self.entregas.get(self.id, 0),
             "v": round(self.cmd[0], 2), "w": round(self.cmd[1], 2),
             "giro": self.est.con_giro, "inv": self.est.signo_invertido,
             "par": self.par_fresco(), "dm": round(self.cal.desfase), "dn": self.cal.mediciones,
             "hz": round(self.hz), "lat": round(self.est.latencia), "esc": [round(x) for x in self.est.escalas],
             "ex": round(getattr(self, "_extra", 0.0), 2)}
        d.update(getattr(self, "diag", None) or {})      # WiFi, memoria, reinicios (code.py)
        if self.cal.ultimo_error is not None:
            d["de"] = round(self.cal.ultimo_error)
        if self.logs:
            d["lg"] = self.logs
        if self.objetivo:
            d["o"] = [round(self.objetivo[0], 1), round(self.objetivo[1], 1)]
        if self.ruta:
            d["r"] = [[round(q[0], 1), round(q[1], 1)] for q in self.ruta[:4]]
        if self.plan:
            d["p"] = dict((str(k), v) for k, v in self.plan.items())
        return d

    def informe_texto(self):
        import json
        d = self.informe()
        if self.pose:
            d["pos"] = [round(self.pose[0], 2), round(self.pose[1], 2)]
        d["k"] = [[round(q[0], 1), round(q[1], 1)] for q in self.mi_corredor()]
        return json.dumps(d)
