# cerebro.py — la lógica autónoma del rover: estima dónde está y hacia dónde
# mira, decide qué cubo le toca, planifica la ruta, controla los motores y se
# coordina con el compañero. NO toca hardware directamente: recibe un objeto
# `hw` (motores, giroscopio, LED) y un objeto `enlace` (radio con el otro
# rover). Por eso el MISMO archivo corre en el CenfoBot y en el simulador.
#
# Se llama a Cerebro.paso() muchas veces por segundo. Nada bloquea: cada
# llamada mira el estado, decide un comando de motores y vuelve.
#
# Máquina de estados de una tarea (un cubo):
#
#   ELEGIR ─► IR ─► ALINEAR ─► CAPTURAR ─► LLEVAR ─► SOLTAR ─► VERIFICAR ─┐
#     ▲        ▲                   │           │                          │
#     │        └───── se escapó ───┴───────────┘                          │
#     └────────────────────────────────────────────────────────────────── ┘
#   ESPERAR:   mi camino se cruza con el del compañero: espero a que pase
#   APARTAR:   estoy en el camino del compañero: me corro
#   LIBRE:     no me toca ningún cubo ahora (me estaciono lejos del compañero)
#   FIN:       no quedan cubos pendientes
#   DESTRABAR: retrocede un poco si quedó trabado o encarado con el compañero
#
# La lógica está repartida en varios archivos (el ESP32 compila cada uno por
# separado, así usa menos memoria):
#   estimador.py    rumbo = giroscopio + cámara
#   coordinacion.py radio, reparto de cubos, caminos reservados, ceder el paso
#   movimiento.py   girar / avanzar / frenar / no chocar
#   pruebas.py      órdenes de prueba del panel e informe de estado
#   arranque.py     comprobar el marcador al arrancar; cubo ajeno en las paletas

import geometria as G
import plan as P
from estimador import Estimador, CalibradorMarcador
from coordinacion import Coordinacion
from movimiento import Movimiento
from pruebas import Pruebas
from arranque import Arranque
from estados import ACTIVOS


class Cerebro(Coordinacion, Movimiento, Pruebas, Arranque):

    def __init__(self, cfg, hw, enlace=None, log=print):
        self.cfg = cfg
        self.hw = hw
        self.enlace = enlace
        self._log_ext = log
        self.logs = []            # últimas líneas del registro (van al panel)
        self.nlog = 0
        self.id = cfg.ROBOT_ID
        self.est = Estimador(getattr(cfg, "LATENCIA_CAMARA_MS", 350),
                             getattr(cfg, "CELDAS_POR_POTENCIA", 30.0))
        self._util = 0.0          # potencia útil que se mandó a los motores (odometría)
        self._obs_ult = None
        self._foto_n = 0          # cuántas fotos nuevas de mí llegaron
        self._edad_rover = 9999   # edad (ms) de mi pose en el último mensaje
        self.pose_cam = None      # (col, row, rumbo) de la última foto, tal cual
        self.cal = CalibradorMarcador(getattr(cfg, "DESFASES_MARCADOR", {}).get(cfg.ROBOT_ID, 0.0))
        self._v_rampa = 0.0       # avance y giro que realmente se mandan (con rampa)
        self._w_rampa = 0.0
        self._rampa_ms = None
        self.hz = 0.0
        self.w = None
        self.w_ms = 0
        self.ult_ms = None
        self.pose = None          # (col, row) de la cámara
        self.pose_ms = 0
        self.prueba = None        # orden del panel (solo MODO_PRUEBAS)
        self.auto_forzado = False
        self._env_ms = 0
        self._dep_ms = 0
        self.reiniciar()

    # ------------------------------------------------------------------ util
    def log(self, txt):
        self.nlog += 1
        self.logs = (self.logs + [[self.nlog, txt[:90]]])[-3:]
        self._log_ext(txt)

    def reiniciar(self):
        self.estado = "ESPERA"
        self.estado_ms = 0
        self.tarea = None
        self.excluir = {}         # color -> ms hasta cuando no elegirlo
        self.entregas = {}        # id -> cubos entregados (regla 12.2.13)
        self.prev_entregado = {}  # color -> bool
        self.par = None           # último mensaje del compañero
        self.par_ms = -100000
        self.plan = None
        self.plan_ms = -100000
        self.ruta = []
        self.ruta_ms = -100000
        self.objetivo = None      # punto al que va (para el panel)
        self.espera_ms = 0
        self.mov_ref = None       # para detectar que quedó trabado
        self.agarre_ms = 0
        self.soltar_desde = None
        self.u_captura = 0.0
        self.motivo = ""
        self.cmd = (0.0, 0.0)
        self._volver_a = None
        self._cal_ini = None
        self._u_malos = {}        # color -> rumbos de captura por los que me trabé
        self._vp = (0.0, 0.0, 0.0)
        self._pp_prev = None

    def cambiar(self, nuevo, ahora, motivo=""):
        if nuevo != self.estado:
            # no llenar la consola con "ESPERAR <-> ELEGIR" repetidos (imprimir
            # por el USB es lento en el ESP32 y frena el control)
            q = ("ESPERAR", "ELEGIR", "LIBRE")
            repetido = self.estado in q and nuevo in q and \
                ahora - getattr(self, "_ult_log_ms", -100000) < 5000
            if not repetido:
                self.log("[{:6.1f}s] {} -> {} {} {}".format(
                    ahora / 1000.0, self.estado, nuevo, self.tarea or "", motivo))
                if self.estado in q and nuevo in q:
                    self._ult_log_ms = ahora
            self.estado = nuevo
            self.estado_ms = ahora
            self.espera_ms = 0
            self.mov_ref = None
            self.ruta_ms = -100000
            self.motivo = motivo

    def _t(self, ahora):
        return (ahora - self.estado_ms) / 1000.0

    def rumbo(self):
        return self.est.rumbo()

    def mi_pose(self):
        return (self.pose[0], self.pose[1], self.rumbo())


    def cubo(self, color):
        c = self.w.cubos.get(color)
        if c is None:
            return None
        return (c[0], c[1])

    def entregado(self, color):
        c = self.w.cubos.get(color)
        z = self.w.zonas.get(color)
        if c is None or z is None:
            return True
        if c[3]:
            return True
        # el árbitro (in_depot) tarda 1 s en confirmarlo; mientras tanto se lo da
        # por entregado solo si está BIEN adentro (con margen: la foto puede
        # estar corrida si las paletas lo tapan a medias)
        falta = G.falta_para_zona((c[0], c[1]), z, self.w.grid, self.w.zona_tam, self.w.lado_cubo + 1.0)
        return falta == 0.0 and c[2] < 1500

    def pendientes(self):
        return [c for c in self.w.cubos if c in self.w.zonas and not self.entregado(c)]

    def cubo_en_paletas(self, pose, color, holgura=1.0):
        c = self.cubo(color)
        if c is None:
            return False
        a, i = G.a_marco_robot(pose, c)
        return (self.cfg.AGARRE - 1.5 <= a <= self.cfg.AGARRE + 1.0 + holgura
                and abs(i) < 1.6 + holgura * 0.4)


    def paso(self, ahora, mundo=None, orden=None):
        """Una vuelta del control. ahora = ms del reloj local."""
        cfg = self.cfg
        self._ahora = ahora
        dt = 0.0 if self.ult_ms is None else (ahora - self.ult_ms) / 1000.0
        if dt > 0.2:
            dt = 0.2
        self.ult_ms = ahora
        self.est.giroscopio(self.hw.giro_dps(), dt)
        self.est.avanzar(self._util, dt)          # odometría con lo que se mandó a los motores
        self.est.registrar(ahora)
        self._medir_giro()
        # si el bucle anda lento (ESP32 cargado), se va más despacio
        self.dt_medio = 0.9 * getattr(self, "dt_medio", dt) + 0.1 * dt
        if self.dt_medio > 0:
            self.hz = 1.0 / self.dt_medio

        if self.enlace is not None:
            for txt in self.enlace.recibir():
                self._mensaje_par(txt, ahora)
        if orden:
            self._orden_prueba(orden, ahora)

        if mundo is not None and mundo.valido:
            self.w = mundo
            self.w_ms = ahora
            r = mundo.rovers.get(self.id)
            obs = mundo.ts_ms - (r[3] if r is not None else 0)
            self._edad_rover = r[3] if r is not None else 9999
            if r is not None and r[3] <= cfg.EDAD_MAX_ROVER_MS and obs != self._obs_ult:
                # foto NUEVA (la visión repite la misma foto muchas veces por segundo)
                self._obs_ult = obs
                th = (r[2] + self.cal.desfase) % 360.0      # marcador -> frente (paletas)
                self.pose_cam = (r[0], r[1], th)
                self._foto_n += 1
                self.pose_ms = ahora - r[3]
                # cuándo se SACÓ la foto, en el reloj del robot (llega ~0,5 s tarde)
                self._foto_t = self.pose_ms - self.est.latencia
                # para saber si la cámara es rápida: cuándo llegaron las últimas fotos
                fr = getattr(self, "_fotos_rec", [])
                fr.append(ahora)
                self._fotos_rec = fr[-6:]
                self.est.foto(r[0], r[1], th, ahora, r[3])
                if self.cal.muestra(
                        ahora - r[3], r[0], r[1], th, self._v_rampa,
                        self.est.hg if self.est.con_giro else None,
                        self.estado == "CALIBRAR" or (self.prueba or {}).get("k") == "CALIBRAR"):
                    self.est.reiniciar_rumbo()
                    self.ruta_ms = -100000
                    self.log("Marcador girado respecto de las paletas: desfase corregido a {:.0f} grados"
                             .format(self.cal.desfase))
            self._contar_entregas()
        p = self.est.posicion()
        if p is not None:
            self.pose = p          # posición AHORA (foto + lo que avanzó desde entonces)
        if self.w is not None:
            try:
                self._seguir_par(ahora)
            except Exception:  # noqa — nunca puede trabar el bucle
                pass

        try:
            v, w = self._decidir(ahora)
        except Exception as ex:  # un error de lógica NO puede dejar los motores andando
            self.log("ERROR en la logica: {!r}".format(ex))
            self.errores = getattr(self, "errores", 0) + 1
            if getattr(self, "relanzar_errores", False):
                raise
            self.reiniciar()
            v, w = 0.0, 0.0
        self._motores(v, w)
        self._anunciar(ahora)
        return v, w

    def _decidir(self, ahora):
        cfg = self.cfg
        if self.w is None or ahora - self.w_ms > cfg.SIN_DATOS_S * 1000:
            self.hw.led((40, 0, 40))
            return 0.0, 0.0                       # sin telemetría: quieto
        fase = self.w.fase
        jugando = fase == cfg.FASE_ARRANQUE or self.auto_forzado
        if self.prueba is not None and not jugando:
            return self._prueba(ahora)
        if not jugando:
            if self.estado != "ESPERA":
                self.log("Fase {}: me detengo".format(fase))
                self.reiniciar()
            self.hw.led((0, 0, 60) if fase == "IDLE" else
                        ((0, 60, 60) if fase == "READY" else (60, 60, 60)))
            return 0.0, 0.0
        if self.pose is None or ahora - self.pose_ms > 2500:
            self.hw.led((60, 20, 0))
            return 0.0, 0.0                       # la cámara no me ve
        if self.estado == "ESPERA":
            if self.cal.confirmado or not getattr(cfg, "CALIBRAR_AL_ARRANCAR", False):
                self.cambiar("ELEGIR", ahora, "arranque")
            else:
                self._cal_volver = "ELEGIR"
                self.cambiar("CALIBRAR", ahora, "arranque: compruebo el marcador")
        return self._maquina(ahora)


    def _maquina(self, ahora):
        cfg = self.cfg
        e = self.estado
        pose = self.mi_pose()

        # chequeos generales de la tarea en curso
        if e in ("IR", "ALINEAR", "CAPTURAR", "LLEVAR", "SOLTAR", "VERIFICAR") and \
                (self.tarea is None or self.tarea not in self.w.cubos or self.tarea not in self.w.zonas):
            self.tarea = None
            self.cambiar("ELEGIR", ahora, "ya no hay cubo para esta tarea")
            return 0.0, 0.0
        if e == "CALIBRAR":
            return self._calibrar(ahora, pose)
        self._vigilar_avance(ahora, pose)
        if self.tarea is not None and e not in ("ELEGIR", "FIN", "LIBRE", "SOLTAR", "VERIFICAR"):
            if self.tarea not in self.w.cubos or self.entregado(self.tarea):
                if e == "LLEVAR":
                    self.cambiar("SOLTAR", ahora, "ya quedo en la zona")
                    return 0.0, 0.0
                self.tarea = None
                if e != "APARTAR":     # apartándome sigo (antes daba vueltas ELEGIR<->APARTAR quieto)
                    self.cambiar("ELEGIR", ahora, "el cubo ya esta entregado")
                    return 0.0, 0.0
        if self.tarea is not None and e not in ("ELEGIR", "FIN", "LIBRE", "SOLTAR", "VERIFICAR"):
            if e in ("IR", "ALINEAR") and (self.lo_lleva_el_par(self.tarea) or self._ceder_al_par()):
                self.excluir[self.tarea] = ahora + 8000
                self.cambiar("ELEGIR", ahora, "lo lleva el companero")
                return 0.0, 0.0
            if self._t(ahora) > cfg.TIEMPO_MAX_TAREA:
                self.excluir[self.tarea] = ahora + 10000
                self.cambiar("ELEGIR", ahora, "tarda demasiado")
                return 0.0, 0.0
            if (e == "IR" and self.par_fresco() and self.par.get("t") == self.tarea
                    and self.par.get("e") in ("IR", "ALINEAR", "CAPTURAR", "LLEVAR")
                    and not self._gano_conflicto()):
                self.excluir[self.tarea] = ahora + 6000
                self.cambiar("ELEGIR", ahora, "el companero va por el mismo")
                return 0.0, 0.0

        if e not in ("APARTAR", "CAPTURAR", "LLEVAR", "SOLTAR") and self._estorbo(pose):
            self.cambiar("APARTAR", ahora, "estoy en el camino del companero")
        elif self._esperar_ir(ahora, e):
            self.cambiar("ESPERAR", ahora, "nuestros caminos se cruzan, pasa el primero")
            return 0.0, 0.0
        if self.estado == "APARTAR":
            return self._apartar(ahora, pose)
        if e == "ELEGIR":
            return self._elegir(ahora)
        if e == "IR":
            return self._ir(ahora, pose)
        if e == "ALINEAR":
            ajeno = self._cubo_ajeno_en_paletas(pose)
            if ajeno is not None:
                self.tarea = ajeno
                self.agarre_ms = ahora
                self.cambiar("LLEVAR", ahora, "se me metio entre las paletas")
                return 0.0, 0.0
            c = self.cubo(self.tarea)
            v, w, listo = self._girar_a(G.rumbo(pose, c), cfg.GIRO_MAX)
            if listo or self._t(ahora) > 4:
                self.cambiar("CAPTURAR", ahora)
            return self._vigilar(ahora, v, w)
        if e == "CAPTURAR":
            return self._capturar(ahora, pose)
        if e == "LLEVAR":
            return self._llevar(ahora, pose)
        if e == "SOLTAR":
            if self.soltar_desde is None:
                self.soltar_desde = (pose[0], pose[1])
            # (con la cámara lenta se para a esperar fotos: tiempo de sobra para
            # que la punta no quede pegada al cubo entregado)
            if G.dist(self.soltar_desde, pose) >= cfg.RETROCESO or self._t(ahora) > 6.0:
                self.soltar_desde = None
                self.cambiar("VERIFICAR", ahora)
                return 0.0, 0.0
            return self._vigilar(ahora, -cfg.VEL_CAPTURA, 0.0)
        if e == "VERIFICAR":
            c = self.w.cubos.get(self.tarea)
            if c is None or self.entregado(self.tarea):
                self.tarea = None
                self.cambiar("ELEGIR", ahora, "entregado")
            elif self._t(ahora) > 3.0 and c[2] < 400:
                # pasó el segundo que pide el árbitro (+ el atraso) y no lo cuenta
                self.cambiar("IR", ahora, "quedo afuera, reintento")
            return 0.0, 0.0
        if e == "DESTRABAR":
            if self._t(ahora) > 1.0:
                self.cambiar(self._volver_a or "ELEGIR", ahora, "destrabado")
                return 0.0, 0.0
            return self._vigilar(ahora, -cfg.VEL_CAPTURA, 0.0, destrabando=True)
        if e == "ESPERAR":
            self.hw.led((60, 30, 0))
            if self._t(ahora) > 0.4:
                self.cambiar("ELEGIR", ahora)
            return 0.0, 0.0
        if e == "LIBRE":
            if self._t(ahora) > 1.0:
                self.cambiar("ELEGIR", ahora)
            return self._estacionar(ahora, pose)
        if e == "FIN":
            self.hw.led((0, 80, 0))
            if self._t(ahora) > 0.5 and self.pendientes():
                self.cambiar("ELEGIR", ahora, "hay un cubo pendiente otra vez")
            return 0.0, 0.0
        self.cambiar("ELEGIR", ahora)
        return 0.0, 0.0


    def _elegir(self, ahora):
        pend = self.pendientes()
        if not pend:
            self.tarea = None
            self.cambiar("FIN", ahora, "no quedan cubos")
            return 0.0, 0.0
        if self.tarea in pend and self.tarea and self.excluir.get(self.tarea, 0) < ahora \
                and self.cubo_en_paletas(self.pose_cam or self.mi_pose(), self.tarea):
            self.cambiar("LLEVAR", ahora, "ya lo tengo")
            return 0.0, 0.0
        p = self._calcular_plan(ahora)
        self.plan = p
        self.plan_ms = ahora
        mia = [c for c in ((p or {}).get(self.id) or []) if not self.lo_lleva_el_par(c)]
        if not mia:
            self.tarea = None
            self.cambiar("LIBRE", ahora, "no me toca ninguno ahora")
            return 0.0, 0.0
        yo = (self.pose[0], self.pose[1])
        k_par = self.corredor_par() if self._par_primero(1) else []
        # Si el compañero todavía no arrancó pero es el líder, su primera tarea
        # (según el mismo plan) también cuenta como reservada.
        if self.pose_par() is not None and self._rango_par() == 0 and self.id_par() < self.id \
                and not (self.par_fresco() and self.par.get("e") in ("ESPERAR", "LIBRE", "FIN", "APARTAR")):
            suya = (p or {}).get(self.id_par()) or []
            if suya:
                pp = self.pose_par()
                k_par = self._corredor_tarea(suya[0], (pp[0], pp[1]))
        # si hace mucho que espero, voy igual (el anti-choques me cuida)
        if getattr(self, "_esperando_desde", None) is None:
            self._esperando_desde = ahora
        # (solo si el compañero no se está moviendo: está trabado, no ocupado)
        pv = self._vp
        quieto = pv[0] * pv[0] + pv[1] * pv[1] < 1.0
        harto = ahora - self._esperando_desde > getattr(self.cfg, "ESPERA_MAX_PAR_MS", 4000)
        # Primero mi tarea del plan. Si su camino cruza el del compañero, en vez
        # de quedarme esperando pruebo otro cubo pendiente cuyo camino NO cruce
        # (y que no sea el que él lleva o va a buscar).
        cands = mia[:1]
        if self.pose_par() is not None and len(pend) >= 2:
            suyo = self.par.get("t") if self.par_fresco() and self.par.get("e") in ACTIVOS else None
            otros = [c for c in pend if c not in cands and c != suyo and self.excluir.get(c, 0) < ahora
                     and not self.lo_lleva_el_par(c)]
            otros.sort(key=lambda c: G.dist(yo, self.cubo(c)) + G.dist(self.cubo(c), self.w.zonas[c]))
            cands = cands + otros
        for n, color in enumerate(cands):
            libre = self._dist_corredores(self._corredor_tarea(color, yo), k_par) >= self.cfg.CORREDOR \
                or self.pose_par() is None
            if libre or (harto and n == 0):
                if libre:
                    self._esperando_desde = None   # (si voy por "harto", el reloj sigue: no vuelvo a esperar)
                self.tarea = color
                self.hw.led((0, 80, 0))
                self.cambiar("IR", ahora, "plan {}{}".format(p, "" if n == 0 else " (voy por " + color + ": no cruza)"))
                return 0.0, 0.0
        self.tarea = None
        self.cambiar("ESPERAR", ahora, "mi camino cruza el del companero")
        return 0.0, 0.0


    def _ir(self, ahora, pose):
        cfg = self.cfg
        # si sin querer agarré otro cubo pendiente, lo llevo a él (si lo suelto
        # girando, lo arrastro por la cancha)
        ajeno = self._cubo_ajeno_en_paletas(pose)
        if ajeno is not None:
            self.tarea = ajeno
            self.agarre_ms = ahora
            self.cambiar("LLEVAR", ahora, "se me metio entre las paletas")
            return 0.0, 0.0
        malos = self._u_malos.get(self.tarea)
        c = self.cubo(self.tarea)
        # ¿ya lo tengo justo delante? atajo directo a capturar
        a, i = G.a_marco_robot(pose, c)
        if cfg.AGARRE - 0.5 <= a <= cfg.AGARRE + cfg.PRE_AGARRE + 1.0 and abs(i) < 1.2 and \
                not (malos and any(abs(G.dif_ang(pose[2], x)) < 25.0 for x in malos)):
            self.cambiar("CAPTURAR", ahora, "atajo")
            return 0.0, 0.0
        obs = self._obstaculos(False)
        # el propio cubo también estorba mientras no estemos detrás de él
        obs_c = obs + [(c, cfg.AGARRE + 0.5)]
        pref = getattr(self, "_u_pref", None)
        pref = pref[1] if pref is not None and pref[0] == self.tarea else None
        previo, u = P.punto_previo(cfg, self.w, self.tarea, obs, malos, pref)
        self._u_pref = (self.tarea, u)
        self.u_captura = u
        if G.dist(pose, previo) < cfg.TOL_PUNTO * 1.5:
            self.cambiar("ALINEAR", ahora)
            return 0.0, 0.0
        v, w = self._seguir_ruta(ahora, pose, previo, cfg.VEL_CRUCERO, obs_c)
        return self._vigilar(ahora, v, w)

    def _capturar(self, ahora, pose):
        cfg = self.cfg
        cw = self.w.cubos[self.tarea]
        c = (cw[0], cw[1])
        a, i = G.a_marco_robot(pose, c)
        if getattr(self, "_cap_key", None) != self.estado_ms:
            self._cap_key = self.estado_ms
            self._cap_malas = 0
            self._cap_rel = None
            self._cap_vistas = []                 # [(col, row) del cubo] en fotos de ESTE intento
        # Lo que dice la FOTO (robot y cubo vistos en el mismo instante) manda;
        # la posición estimada solo sirve cuando el cubo queda tapado.
        rel = self._rel_foto(self.tarea)
        if rel is not None:
            ac, ic = rel
            self._cap_rel = rel
            if ac <= cfg.AGARRE + 1.5 and abs(ic) < 2.6:
                self.agarre_ms = ahora
                self.cambiar("LLEVAR", ahora, "agarrado")
                return self._llevar(ahora, pose)  # sin frenar: sigue empujando hacia la zona
            malo = ac < cfg.AGARRE - 3.0 or abs(ic) > 3.2 or ac > cfg.AGARRE + cfg.PRE_AGARRE + 6
            self._cap_malas = self._cap_malas + 1 if malo else 0
            if self._cap_malas >= 2:             # dos fotos seguidas: se escapó de verdad
                self.cambiar("IR", ahora, "se escapo al capturar")
                return 0.0, 0.0
            # ¿lo vengo empujando con la PUNTA de una paleta? Se decide solo con
            # fotos: el cubo se corrió bastante y sigue adelante y de costado.
            # (Antes se usaba lo que el robot creía haber avanzado, y con la
            # cámara lenta eso fallaba: retrocedía y volvía a empujar sin parar.)
            vs = self._cap_vistas
            vs.append((cw[0], cw[1]))
            if len(vs) >= 2 and G.dist(vs[0], vs[-1]) > 2.0 and ac > cfg.AGARRE + 1.5 and abs(ic) >= 1.6:
                self._volver_a = "IR"
                self.cambiar("DESTRABAR", ahora, "lo empujaba con la punta: me acomodo")
                return 0.0, 0.0
        cerca = self._cap_rel is not None and self._cap_rel[0] <= cfg.AGARRE + 6 and abs(self._cap_rel[1]) < 2.5
        if a <= cfg.AGARRE + 0.3 and abs(i) < 1.6 and cerca and cw[2] > 350:
            self.agarre_ms = ahora                # tapado por las paletas: lo doy por agarrado
            self.cambiar("LLEVAR", ahora, "agarrado (tapado)")
            return self._llevar(ahora, pose)
        # el cubo ya quedó dentro de su zona (lo empujé hasta ahí): no lo saco
        zt = self.w.zonas.get(self.tarea)
        if zt is not None and cw[2] < 350 and \
                G.falta_para_zona(c, zt, self.w.grid, self.w.zona_tam, self.w.lado_cubo + 0.6) == 0.0:
            self.cambiar("SOLTAR", ahora, "el cubo ya entro a su zona")
            return 0.0, 0.0
        if self._t(ahora) > 10:
            self.cambiar("IR", ahora, "tarda en capturar" if cw[2] < 3000 else
                         "la camara no ve el cubo hace {:.0f} s (calibrar colores)".format(cw[2] / 1000.0))
            return 0.0, 0.0
        err = G.dif_ang(G.rumbo(pose, c), pose[2])
        w = G.limitar(cfg.KP_RUMBO * 1.5 * err, -0.3, 0.3)
        v = cfg.VEL_CAPTURA if abs(err) < 25 else 0.0
        self.objetivo = P.objetivo_zona(cfg, self.w, self.tarea)    # después de agarrarlo voy hacia allá
        return self._vigilar(ahora, v, w)

    def _estacionar(self, ahora, pose):
        """Sin tarea: me corro hacia un costado del lado de la salida, lejos del compañero."""
        cfg = self.cfg
        s = self.w.salida
        pp = self.pose_par()
        opciones = [(s[0] + 3.0, 7.0), (s[0] + 3.0, self.w.rows - 7.0)]
        if pp is not None:
            k = self.corredor_par()
            opciones.sort(key=lambda q: -min(G.dist(q, (pp[0], pp[1])), self._dist_corredores([q], k)))
        dest = opciones[0]
        if G.dist(pose, dest) < 2.0:
            return 0.0, 0.0
        v, w = self._seguir_ruta(ahora, pose, dest, cfg.VEL_CRUCERO * 0.8, self._obstaculos(False))
        return self._vigilar(ahora, v, w)
