# prueba_sin_camara.py — pruebas del robot que NO necesitan la cámara.
#
# Se sube con:   python herramientas/subir_robot.py 10 --prueba
# (queda en el robot con el nombre code.py y arranca solo).
# Se usa desde la consola de Thonny: va diciendo qué hacer y espera Enter.
#
# Al final imprime los valores para poner en firmware/rover/robot_10.py (u 11).
# Para volver al programa de competencia:  python herramientas/subir_robot.py 10

import time
import config as cfg
from hw import Hardware

hw = Hardware(cfg)
res = {}


def pausa(texto):
    hw.parar()
    print()
    print(">>> " + texto)
    input("    (Enter para seguir) ")


def preguntar(texto):
    while True:
        r = input(">>> " + texto + " [s/n]: ").strip().lower()
        if r in ("s", "n"):
            return r == "s"


def integrar_giro(segundos, motores=None):
    """Suma lo que gira el robot (grados, + = izquierda/antihorario) durante 'segundos'."""
    ang = 0.0
    t0 = time.monotonic()
    prev = t0
    ult_print = t0
    while time.monotonic() - t0 < segundos:
        if motores:
            if motores(time.monotonic() - t0, ang) is False:
                break
        dps = hw.giro_dps()
        ahora = time.monotonic()
        if dps is not None:
            ang += dps * (ahora - prev)
        prev = ahora
        if ahora - ult_print > 0.5:
            print("    giro acumulado: {:7.1f} grados".format(ang))
            ult_print = ahora
        time.sleep(0.005)
    hw.parar()
    return ang


print()
print("=" * 50)
print(" PRUEBAS SIN CAMARA - robot ID", cfg.ROBOT_ID)
print("=" * 50)

# ------------------------------------------------------------ 1. luz
print("\n[1] LUZ: rojo, verde, azul")
for c in ((80, 0, 0), (0, 80, 0), (0, 0, 80), (0, 0, 0)):
    hw.led(c)
    time.sleep(0.5)
res["luz"] = preguntar("¿Viste la luz cambiar de color?")

# ------------------------------------------------------------ 2. giroscopio
if hw.imu is None:
    print("\n[2] GIROSCOPIO: NO se encontró la IMU. Revisá el cable Qwiic.")
    res["imu"] = False
else:
    res["imu"] = True
    pausa("[2] GIROSCOPIO. Dejá el robot QUIETO sobre la mesa.")
    hw.calibrar_giro(2.0)
    quieto = integrar_giro(5)
    print("    En 5 s quieto se 'movió' {:.1f} grados (ideal: menos de 1).".format(quieto))
    pausa("Ahora VOS vas a girar el robot CON LA MANO: cuando aprietes Enter, "
          "tenés 6 segundos para darle un cuarto de vuelta (90 grados) hacia la IZQUIERDA "
          "(contra las agujas del reloj, mirando desde arriba). Agarralo antes de apretar Enter.")
    print("    ¡YA! GIRALO AHORA...")
    hw.led((0, 0, 80))
    g = integrar_giro(6)
    hw.led((0, 0, 0))
    print("    Medido: {:.1f} grados".format(g))
    if abs(g) < 20:
        print("    Casi no detectó giro: ¿lo giraste? Repetí la prueba más tarde.")
    elif g > 0:
        print("    BIEN: girar a la izquierda da positivo.")
        res["GIRO_SIGNO"] = cfg.GIRO_SIGNO
    else:
        print("    Dio NEGATIVO: hay que invertir GIRO_SIGNO.")
        cfg.GIRO_SIGNO = -cfg.GIRO_SIGNO
        res["GIRO_SIGNO"] = cfg.GIRO_SIGNO

# ------------------------------------------------------------ 3. ruedas
pausa("[3] RUEDAS. LEVANTÁ el robot (que las ruedas no toquen nada). "
      "Primero se mueve SOLO la rueda IZQUIERDA (mirando el robot desde atrás) "
      "hacia ADELANTE, 1,5 segundos.")
hw.motores(0.5, 0.0)
time.sleep(1.5)
hw.parar()
izq_mueve = preguntar("¿Se movió la rueda IZQUIERDA (y no la derecha)?")
izq_adelante = preguntar("¿Giró hacia ADELANTE? (ADELANTE = el lado de las PALETAS: la rueda "
                         "tiene que girar como cuando el robot avanza hacia las paletas)")
pausa("Ahora SOLO la rueda DERECHA hacia adelante, 1,5 segundos.")
hw.motores(0.0, 0.5)
time.sleep(1.5)
hw.parar()
der_mueve = preguntar("¿Se movió la rueda DERECHA (y no la izquierda)?")
der_adelante = preguntar("¿Giró hacia ADELANTE (hacia las paletas)?")
res["MOTOR_IZQ_SIGNO"] = cfg.MOTOR_IZQ_SIGNO * (1.0 if izq_adelante else -1.0)
res["MOTOR_DER_SIGNO"] = cfg.MOTOR_DER_SIGNO * (1.0 if der_adelante else -1.0)
cfg.MOTOR_IZQ_SIGNO = res["MOTOR_IZQ_SIGNO"]
cfg.MOTOR_DER_SIGNO = res["MOTOR_DER_SIGNO"]
if not (izq_mueve and der_mueve):
    print("    ¡OJO! Las ruedas están cruzadas (motor 1 debería ser la izquierda).")
    print("    Avisale a Claude: hay que intercambiar motores en el código.")
    res["ruedas_cruzadas"] = True

# ------------------------------------------------------------ 4. en el piso
if res.get("imu"):
    pausa("[4] EN EL PISO. Poné el robot en el piso con 50 cm libres alrededor. "
          "Va a buscar la potencia mínima con la que empieza a girar.")
    hw.calibrar_giro(1.5)
    minimo = None
    p = 0.10
    while p <= 0.6 and minimo is None:
        hw.motores(-p, p)
        t0 = time.monotonic()
        mov = 0.0
        while time.monotonic() - t0 < 0.6:
            d = hw.giro_dps()
            if d is not None and abs(d) > mov:
                mov = abs(d)
            time.sleep(0.01)
        hw.parar()
        print("    potencia {:.2f} -> {:.0f} grados/s".format(p, mov))
        if mov > 25:
            minimo = p
        p += 0.03
        time.sleep(0.3)
    if minimo:
        res["MOTOR_MINIMO"] = round(min(0.45, minimo + 0.02), 2)
        print("    Empieza a moverse con {:.2f}".format(minimo))

    pausa("Ahora gira 90 grados a la IZQUIERDA usando el giroscopio.")
    hw.calibrar_giro(1.0)

    def girar(t, ang, objetivo=90.0):
        # igual que el programa de competencia: frena al acercarse y no pasa de 0,3
        falta = objetivo - ang
        minimo = res.get("MOTOR_MINIMO", cfg.MOTOR_MINIMO)
        pot = minimo + (0.30 - minimo) * min(1.0, abs(falta) / 40.0)
        s = 1 if falta > 0 else -1
        if ang < -25:
            print("    ¡ALTO! Gira para el lado CONTRARIO al pedido: los motores están al revés.")
            res["giro_al_reves"] = True
            return False
        if abs(falta) < 2:
            hw.parar()
        else:
            hw.motores(-pot * s, pot * s)
    g = integrar_giro(4, girar)
    time.sleep(0.5)
    print("    Terminó en {:.1f} grados (objetivo 90).".format(g))
    res["giro_90"] = round(g, 1)
    res["giro_ok"] = preguntar("¿El robot quedó mirando más o menos 90° a la izquierda de donde empezó?")
    if not res["giro_ok"] and res.get("GIRO_SIGNO", 1.0) == 1.0:
        print("    Si giró mucho más o mucho menos, anotalo: después se ajusta GIRO_ESCALA.")

    pausa("[5] RECTO 2 segundos (con 1 m libre adelante). Mido si se tuerce.")
    hw.calibrar_giro(1.0)

    def recto(t, ang):
        hw.motores(0.45, 0.45)
    desvio = integrar_giro(2.0, recto)
    print("    Se torció {:.1f} grados ({}).".format(
        desvio, "a la izquierda" if desvio > 0 else "a la derecha"))
    if not preguntar("¿Fue hacia ADELANTE (hacia donde apuntan las paletas)?"):
        print("    Fue hacia ATRÁS: los dos motores están invertidos en el código.")
        print("    -> MOTOR_IZQ_SIGNO y MOTOR_DER_SIGNO hay que invertirlos (avisale a Claude).")
        res["MOTOR_IZQ_SIGNO"] = -cfg.MOTOR_IZQ_SIGNO
        res["MOTOR_DER_SIGNO"] = -cfg.MOTOR_DER_SIGNO
        desvio = -desvio
    cm = input(">>> ¿Cuántos centímetros avanzó más o menos? (número, o Enter si no sabés): ").strip()
    try:
        v = float(cm.replace(",", ".")) / 2.0 / 2.0      # cm -> celdas (2 cm) -> por segundo
        res["celdas_por_s_a_0.45"] = round(v, 1)
    except ValueError:
        pass
    # se tuerce a la izquierda => la derecha empuja más => bajar ganancia derecha
    ajuste = 1.0 - desvio * 0.004
    res["GANANCIA_DER"] = round(cfg.GANANCIA_DER * max(0.8, min(1.2, ajuste)), 3)

# ------------------------------------------------------------ 6. red
print()
if preguntar("[6] ¿Probamos el WiFi y la conexión con la compu? (necesita secretos.py bien llenado)"):
    import wifi
    import socketpool
    import red
    ok = red.conectar_wifi(cfg.WIFI_SSID, cfg.WIFI_PASSWORD, hw, intentos=5)
    res["wifi"] = ok
    if ok:
        print("    MAC de este robot:", red.mi_mac())
        pool = socketpool.SocketPool(wifi.radio)
        cli = red.ClienteVision(pool, cfg.VISION_HOST, cfg.VISION_PORT)
        print("    Probando la conexión con {}:{} (tiene que estar corriendo el simulador "
              "o la visión en esa compu)...".format(cfg.VISION_HOST, cfg.VISION_PORT))
        if cli.conectar():
            import mundo
            n = 0
            t0 = time.monotonic()
            while n < 3 and time.monotonic() - t0 < 5:
                l = cli.leer()
                if l:
                    m = mundo.interpretar(l, 0)
                    if m:
                        n += 1
                        print("    Llegó telemetría: fase {}, rovers {}, cubos {}".format(
                            m.fase, list(m.rovers.keys()), list(m.cubos.keys())))
                time.sleep(0.05)
            res["telemetria"] = n > 0
            cli.cerrar()
        else:
            res["telemetria"] = False

        # ------------------------------------------------------- 7. radio
        if preguntar("[7] ¿Probamos la radio ESP-NOW? (corré esta prueba en los DOS robots a la vez)"):
            import enlace
            e = enlace.crear(cfg, pool)
            if e:
                print("    Mandando y escuchando 15 segundos...")
                vistos = 0
                t0 = time.monotonic()
                ult = 0
                while time.monotonic() - t0 < 15:
                    if time.monotonic() - ult > 0.5:
                        e.enviar('{"q":"DYN","id":%d,"e":"PRUEBA"}' % cfg.ROBOT_ID)
                        ult = time.monotonic()
                    for txt in e.recibir():
                        if '"id":%d' % cfg.ROBOT_ID not in txt:
                            vistos += 1
                            if vistos <= 3:
                                print("    Recibido del compañero:", txt)
                    time.sleep(0.02)
                print("    Mensajes recibidos del otro robot:", vistos)
                res["radio"] = vistos > 0

# ------------------------------------------------------------ resumen
hw.parar()
hw.led((0, 60, 0))
print()
print("=" * 50)
print(" RESULTADOS robot", cfg.ROBOT_ID)
print("=" * 50)
for k in sorted(res):
    print("  {:16s} {}".format(k, res[k]))
print()
print("Copiá estas líneas en firmware/rover/robot_{}.py (o pasáselas a Claude):".format(cfg.ROBOT_ID))
for k in ("MOTOR_IZQ_SIGNO", "MOTOR_DER_SIGNO", "GANANCIA_DER", "GIRO_SIGNO", "MOTOR_MINIMO"):
    if k in res:
        print("  {} = {}".format(k, res[k]))
print()
print("Listo. Para volver al programa de competencia: python herramientas/subir_robot.py",
      cfg.ROBOT_ID)
while True:
    time.sleep(1)
