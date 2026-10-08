# code.py — programa de COMPETENCIA del CenfoBot (Vision Rover Challenge).
# Equipo DynamiteCoders. CircuitPython corre este archivo solo al encender.
#
# Qué hace, en orden:
#   1. Luz BLANCA: arranca. Luz ROJA 2 s: mide el giroscopio -> NO MOVER EL ROBOT.
#   2. Luz NARANJA titilando: conectando al WiFi.
#   3. Se conecta a la cámara oficial (TCP 2026) y a la radio con el compañero.
#   4. Espera. Luz AZUL = IDLE, CELESTE = READY (preparación).
#   5. Cuando la cámara pasa a RUNNING, arranca SOLO (reglamento 9 y 11:
#      nadie toca nada). Verde = trabajando, celeste = llevando un cubo,
#      naranja = esperando al compañero, violeta = sin telemetría.
#   6. FINISHED o sin cubos pendientes: se detiene.
#
# Los archivos que tienen que estar en la raíz de CIRCUITPY los copia
# herramientas/subir_robot.py. No hace falta tocarlos a mano.

import time
import gc

import config as cfg
from hw import Hardware


def libre():
    return gc.mem_free() if hasattr(gc, "mem_free") else -1


def ahora_ms():
    try:
        return time.monotonic_ns() // 1000000
    except AttributeError:
        return int(time.monotonic() * 1000)


hw = Hardware(cfg)
hw.led((60, 60, 60))
print()
print("=" * 44)
print(" CenfoBot ID {}  -  DynamiteCoders".format(cfg.ROBOT_ID))
print(" Vision: {}:{}   radio: {}   pruebas: {}".format(
    cfg.VISION_HOST, cfg.VISION_PORT, cfg.ENLACE, cfg.MODO_PRUEBAS))
print("=" * 44)
time.sleep(0.5)
hw.led((80, 0, 0))
hw.calibrar_giro(2.0)

import wifi            # noqa: E402
import socketpool      # noqa: E402
import red             # noqa: E402
import enlace as E     # noqa: E402

REDES = list(getattr(cfg, "WIFI_REDES", None) or [(cfg.WIFI_SSID, cfg.WIFI_PASSWORD)])
red.conectar_wifi(REDES, None, hw)
print("Mi MAC:", red.mi_mac())
pool = socketpool.SocketPool(wifi.radio)
radio = E.crear(cfg, pool)
panel = red.CanalPanel(pool, cfg)
vision = red.ClienteVision(pool, cfg.VISION_HOST, cfg.VISION_PORT)
gc.collect()

# Se importan de a uno, limpiando memoria entre medio (el ESP32 tiene poca).
import geometria       # noqa: E402,F401
gc.collect()
import mundo           # noqa: E402
gc.collect()
import plan            # noqa: E402,F401
gc.collect()
import estimador       # noqa: E402,F401
gc.collect()
import coordinacion    # noqa: E402,F401
gc.collect()
import movimiento      # noqa: E402,F401
gc.collect()
import pruebas         # noqa: E402,F401
gc.collect()
import arranque        # noqa: E402,F401
gc.collect()
from cerebro import Cerebro   # noqa: E402
gc.collect()
print("Memoria libre:", libre())

cerebro = Cerebro(cfg, hw, radio, print)
ultimo_dato = ahora_ms()
fase_impresa = None
vueltas = 0
t_vueltas = time.monotonic()
sin_vision_ms = None
wifi_rehecho = False
m_ult = None
t_udp = -100000
ult_clave = None
ult_parse = 0
tuvo_vision = False        # solo se reinicia solo si ANTES llegó a tener visión

# Por qué se reinició la placa la última vez (WATCHDOG = el perro guardián la
# reinició porque el programa se quedó trabado).
try:
    import microcontroller
    motivo_reinicio = str(microcontroller.cpu.reset_reason).split(".")[-1]
except Exception:  # noqa
    microcontroller = None
    motivo_reinicio = "?"
print("Motivo del ultimo reinicio:", motivo_reinicio)

# PERRO GUARDIÁN: si el programa se queda trabado más de 8 s (pasó en casa:
# la radio WiFi se colgaba con los motores andando y el robot seguía girando
# solo), la placa se reinicia sola y vuelve a arrancar. Al reiniciarse, los
# motores se apagan.
perro = None
try:
    from watchdog import WatchDogMode
    perro = microcontroller.watchdog
    perro.timeout = 8
    perro.mode = WatchDogMode.RESET
    perro.feed()
    print("Perro guardian activo (8 s)")
except Exception as e:  # noqa
    perro = None
    print("Sin perro guardian:", e)


def alimentar():
    if perro is not None:
        perro.feed()


def diagnostico():
    cerebro.diag = {"rssi": red.senal(), "cortes": vision.cortes, "err": vision.error,
                    "mem": libre(), "rr": motivo_reinicio}


def informe_sin_vision():
    d = cerebro.diag
    return '{"id": %d, "e": "SIN_VISION", "rssi": %s, "cortes": %d, "err": "%s", "mem": %d, "rr": "%s"}' % (
        cfg.ROBOT_ID, d.get("rssi") if d.get("rssi") is not None else "null", d.get("cortes", 0),
        str(d.get("err", "")).replace('"', "'"), d.get("mem", -1), d.get("rr", "?"))


diagnostico()
t_diag = time.monotonic()

try:
    while True:
        alimentar()
        try:
            t = ahora_ms()
            if time.monotonic() - t_diag > 1.0:
                t_diag = time.monotonic()
                gc.collect()               # ordena la memoria de a poco (evita que se fragmente)
                diagnostico()
            if not wifi.radio.connected:
                hw.parar()
                vision.cerrar("se cayo el WiFi")
                red.conectar_wifi(REDES, None, hw, alimentar=alimentar)
                continue
            # Lo que manda el panel (solo en pruebas): órdenes y, además, la
            # telemetría reenviada por UDP. En casa la conexión TCP con la
            # visión se traba seguido con el WiFi flojo; la copia por UDP
            # no se traba y el robot sigue trabajando con ella.
            orden = panel.leer_orden()
            linea_udp = None
            if panel.tele is not None:
                linea_udp = panel.tele
                panel.tele = None
                t_udp = t
            udp_ok = t - t_udp < 1500
            if not vision.conectado and not udp_ok:
                hw.parar()
                hw.led((40, 0, 40))
                if sin_vision_ms is None:
                    sin_vision_ms = t
                panel.informar(t, informe_sin_vision)
                if panel.origen and panel.origen != vision.host and vision.fallos >= 1:
                    print("La vision no esta en {}: pruebo con la compu del panel {}".format(
                        vision.host, panel.origen))
                    vision.host = panel.origen
                    vision.fallos = 0
                    vision._ult_intento = -10.0
                if vision.conectar():
                    ultimo_dato = t
                    sin_vision_ms = None
                    wifi_rehecho = False
                    tuvo_vision = True
                elif vision.fallos >= 2 and not wifi_rehecho:
                    # El WiFi dice "conectado" pero perdió la dirección (error
                    # "Name or service not known" en las pruebas): se vuelve a
                    # conectar a la red desde cero, sin reiniciar la placa.
                    wifi_rehecho = True
                    print("Rehago la conexion WiFi")
                    try:
                        wifi.radio.enabled = False
                        time.sleep(0.3)
                        wifi.radio.enabled = True
                    except Exception as e:  # noqa
                        print("  no pude apagar la radio:", e)
                    alimentar()
                    red.conectar_wifi(REDES, None, hw, intentos=1, alimentar=alimentar)
                    vision.fallos = 0
                elif t - sin_vision_ms > 12000 and microcontroller is not None and tuvo_vision:
                    # 12 s sin poder volver a conectarse: la radio del ESP32 quedó
                    # mal. Antes había que apagar y prender el robot a mano; ahora
                    # se reinicia solo (tarda unos 8 s en volver).
                    print("No puedo volver a conectarme a la vision: reinicio la placa")
                    hw.parar()
                    time.sleep(0.3)
                    microcontroller.reset()
                continue
            sin_vision_ms = None
            linea = vision.leer()
            if linea:
                ultimo_dato = t
            elif vision.conectado and not udp_ok and t - ultimo_dato > 2500:
                # conectado pero mudo: la conexión quedó colgada (pasa con WiFi flojo).
                # Se cierra y se vuelve a abrir en vez de quedarse esperando.
                print("Hace 2,5 s que no llegan datos de la vision: reconecto")
                hw.parar()
                vision.cerrar("2,5 s sin datos")
                ultimo_dato = t
                continue
            if not linea and linea_udp and t - ultimo_dato > 300:
                linea = linea_udp         # el TCP no trae nada: uso la copia del panel
            m = None
            if linea:
                # La visión repite la MISMA foto muchas veces por segundo: si es
                # repetida, se reusa la ya leída (menos basura en la memoria del
                # ESP32, que se llenaba y trababa el WiFi).
                k = linea.find(b'"ts_ms"')
                clave = linea[k:k + 26] if k >= 0 else None
                if clave is not None and clave == ult_clave and m_ult is not None and t - ult_parse < 1000:
                    m = m_ult
                else:
                    m = mundo.interpretar(linea, t)
                    if m is not None:
                        m_ult, ult_clave, ult_parse = m, clave, t
            if m is not None and m.fase != fase_impresa:
                print("[fase] {}  (reloj {} s)".format(m.fase, m.transcurrido_ms // 1000))
                fase_impresa = m.fase
            cerebro.paso(t, m, orden)
            panel.informar(t, cerebro.informe_texto)
            vueltas += 1
            if time.monotonic() - t_vueltas > 10:
                print("[info] {:.0f} vueltas/s, mensajes de vision: {}, memoria libre: {}, senal: {} dBm".format(
                    vueltas / (time.monotonic() - t_vueltas), vision.recibidas, libre(), red.senal()))
                vueltas = 0
                t_vueltas = time.monotonic()
            time.sleep(0.001)      # deja respirar al WiFi
        except MemoryError:
            hw.parar()
            vision.error = "sin memoria"
            gc.collect()
        except Exception as e:  # noqa — pase lo que pase, los motores se paran
            hw.parar()
            print("ERROR en el bucle:", repr(e))
            vision.error = "bucle: {!r}".format(e)[:60]
            time.sleep(0.2)
finally:
    # Ctrl+C (Thonny o subir_robot.py): apagar motores y el perro guardián,
    # si no, la placa se reiniciaría sola en medio de la carga de archivos.
    hw.parar()
    if perro is not None:
        try:
            perro.deinit()
        except Exception:  # noqa
            pass
