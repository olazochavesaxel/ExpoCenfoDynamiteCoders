# main.py
#
# Punto de entrada de la estrategia de competencia.
#
# IMPORTANTE: renombrar este archivo a "code.py" al subirlo con Thonny (ver
# README.md de esta carpeta) -- CircuitPython solo corre solo el archivo que
# se llame exactamente code.py.
#
# Secuencia que exige el reglamento (seccion 9): conectarse al WiFi y al
# sistema de vision, esperar a que la vision cambie de fase a READY, y
# arrancar la estrategia SOLA, sin boton ni intervencion humana.

import time
from ideaboard import IdeaBoard

import config
import vision_client
import coordination
import motion
import strategy

ib = IdeaBoard()
ib.pixel = (255, 140, 0)  # naranja: arrancando / conectando

print("=== CenfoBot", config.ROBOT_ID, "===")

vision_client.connect_wifi(config.WIFI_SSID, config.WIFI_PASSWORD)

canal = coordination.current_wifi_channel()

coordinator = coordination.Coordinator(
    config.ROBOT_ID, config.PEER_MAC, channel=canal
)

vc = vision_client.VisionClient(config.VISION_HOST, config.VISION_PORT)
while not vc.ensure_connected():
    time.sleep(1)

ib.pixel = (0, 0, 255)  # azul: conectado, esperando READY
print("Esperando fase READY del sistema de vision...")

fase_anterior = None

while True:
    coordinator.recibir_pendientes()

    if not vc.connected:
        vc.ensure_connected()
        time.sleep(0.2)
        continue

    msg = vc.poll()
    if msg is None:
        time.sleep(0.02)
        continue

    fase = msg["phase"]
    if fase != fase_anterior:
        print("Fase:", fase)
        fase_anterior = fase

    # Si nos conectamos tarde y ya esta RUNNING, arrancamos igual: el
    # reglamento no da forma de "esperar al proximo READY" desde el robot,
    # y quedarse esperando IDLE/READY para siempre seria peor.
    if fase == "READY" or fase == "RUNNING":
        break

    time.sleep(0.02)

ib.pixel = (0, 255, 0)  # verde: arrancando estrategia
print("Arrancando estrategia")

strategy.ciclo_principal(vc, coordinator, config.ROBOT_ID)

motion.detener()
ib.pixel = (0, 0, 0)
print("Fin")
