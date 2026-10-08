# config.py — TODOS los números ajustables del rover, en un solo lugar.
#
# Este archivo es IGUAL para los dos robots. Lo que cambia entre robots vive
# en robot.py (ID del marcador y calibraciones de ESE robot), y la red WiFi
# en secretos.py (no se sube a GitHub). El script herramientas/subir_robot.py
# copia la versión correcta de cada uno.
#
# Unidades: distancias en CELDAS (1 celda = 20 mm), ángulos en grados,
# tiempos en segundos salvo que diga _MS, motores de -1.0 a 1.0.

# ---------------------------------------------------------------------------
# Identidad y calibración de ESTE robot (robot.py). Valores por defecto por
# si robot.py falta: así el robot igual arranca y avisa.
# ---------------------------------------------------------------------------
ROBOT_ID = 10
MOTOR_IZQ_SIGNO = 1.0      # -1.0 si la rueda izquierda va al revés
MOTOR_DER_SIGNO = 1.0      # -1.0 si la rueda derecha va al revés
GANANCIA_DER = 1.0         # RIGHT_GAIN de motor_calibration.py (multiplica motor_2)
GIRO_SIGNO = 1.0           # +1: gyro Z positivo = antihorario (igual que theta)
GIRO_ESCALA = 1.0          # 360 / grados medidos en una vuelta real
MOTOR_MINIMO = 0.20        # throttle mínimo que logra mover el robot (fricción)
# Marcador ArUco pegado girado: grados que hay que SUMARLE al theta de la
# cámara para que apunte a las paletas. Van los de LOS DOS robots (cada uno
# necesita también el del compañero para no chocarlo). El robot lo comprueba
# solo al arrancar (avanza recto unas celdas) y lo corrige si no coincide;
# el botón "Calibrar marcador" del panel lo mide y muestra.
DESFASES_MARCADOR = {10: 0.0, 11: 0.0}   # medidos el 3-oct-2026: los dos 0°
# True = al arrancar la ronda avanza recto unas celdas para comprobar el
# marcador. Los dos ya están medidos, así que no hace falta.
CALIBRAR_AL_ARRANCAR = False
try:
    from robot import *    # noqa: F401,F403
except ImportError:
    print("AVISO: falta robot.py, uso ROBOT_ID =", ROBOT_ID)

# ---------------------------------------------------------------------------
# Red (secretos.py). Nunca 127.0.0.1: es la IP de la compu que corre la visión.
# ---------------------------------------------------------------------------
WIFI_SSID = "PONER_RED"
WIFI_PASSWORD = "PONER_CLAVE"
VISION_HOST = "192.168.1.100"
VISION_PORT = 2026
DEPURACION_HOST = None     # IP de la compu con herramientas/panel.py, o None
DEPURACION_PUERTO = 2028   # el robot le manda su estado al panel (solo informa)
PRUEBAS_PUERTO = 2027      # el panel le manda órdenes de PRUEBA al robot

# MODO_PRUEBAS = True permite que el panel le dé órdenes al robot (ir a un
# punto, girar, capturar un cubo...). Es SOLO para probar en casa.
# En la competencia DEBE estar en False: así el robot ni siquiera escucha
# órdenes externas (reglamento 6.3 y 11.2).
MODO_PRUEBAS = False
# TELE_UDP = True: el robot acepta la COPIA de la telemetría oficial que reenvía
# herramientas/panel.py por UDP (no acepta órdenes). Sirve cuando el TCP con la
# visión se traba (WiFi de 2,4 GHz saturado). En casa lo hacía el modo pruebas.
TELE_UDP = False
# (secretos.py puede cambiar MODO_PRUEBAS y DEPURACION_HOST para probar en casa)
try:
    from secretos import *  # noqa: F401,F403
except ImportError:
    print("AVISO: falta secretos.py (WiFi e IP de la vision)")

# ---------------------------------------------------------------------------
# Comunicación entre rovers
# ---------------------------------------------------------------------------
ENLACE = "espnow"          # "espnow" (recomendado por la organización) o "udp"
ENLACE_EQUIPO = "DYN"      # etiqueta para ignorar mensajes de otros equipos
ENLACE_PERIODO = 0.2       # cada cuánto anunciamos nuestro estado y posición (s)
ENLACE_UDP_PUERTO = 2029

# ---------------------------------------------------------------------------
# Geometría física del CenfoBot (en celdas, medida desde el punto que publica
# la visión, que está casi sobre el eje de las ruedas).
#   AGARRE: distancia hasta el centro del cubo cuando el cubo está metido
#           entre las paletas, tocando el frente. Se mide con el panel
#           (botón "Medir AGARRE", ver docs/GUIA.md).
#   PUNTA:  distancia hasta la punta de las paletas.
# ---------------------------------------------------------------------------
AGARRE = 5.0
PUNTA = 7.0
MEDIO_ANCHO = 3.0          # mitad del ancho del robot con paletas
MEDIO_ANCHO_PAR = 3.0      # ídem en la PUNTA de las paletas (se abren): para no chocar al compañero
COLA = 2.0                 # cuánto sobresale el robot hacia atrás del punto

# ---------------------------------------------------------------------------
# Movimiento
# ---------------------------------------------------------------------------
# Estos valores son la potencia POR ENCIMA del mínimo (MOTOR_MINIMO): el motor
# recibe MOTOR_MINIMO + (1 - MOTOR_MINIMO) * valor. Los motores son rápidos
# (con 0,2 ya va a ~16 cm/s) y la cámara es lenta: por eso valores bajos.
VEL_CRUCERO = 0.27         # avanzando libre
VEL_LLEVANDO = 0.21        # con el cubo entre las paletas
VEL_CAPTURA = 0.16         # los últimos centímetros hacia el cubo
VEL_FINAL = 0.14           # entrando a la zona de acopio
GIRO_MAX = 0.25            # giro en el lugar, libre (más bajo = giros más suaves)
GIRO_MAX_LLEVANDO = 0.22   # giro en el lugar con cubo
KP_GIRO = 0.008            # throttle por grado de error al girar en el lugar
ANTICIPO_GIRO = 0.12       # s: al girar, afloja antes según lo rápido que ya gira (no se pasa)
ACEL_AVANCE = 1.5          # cuánto puede subir el avance por segundo (arranque suave)
ACEL_GIRO = 2.5            # cuánto puede subir el giro por segundo
                           # (frenar siempre es inmediato)
KP_RUMBO = 0.010           # corrección de rumbo mientras avanza
TOL_GIRO = 4.0             # grados para dar un giro por terminado
GIRAR_EN_LUGAR = 40.0      # si el error es mayor, primero gira y después avanza
TOL_PUNTO = 0.8            # celdas para dar un punto por alcanzado
FRENADO = 4.0              # celdas antes del destino en que empieza a frenar

# Cámara lenta: en casa la visión saca ~2,6 fotos por segundo y cada una
# llega ~0,35 s tarde. El robot estima dónde está entre foto y foto con el
# giroscopio y los motores, y mide solo el atraso real (estos son el arranque).
LATENCIA_CAMARA_MS = 600
CELDAS_POR_POTENCIA = 30.0  # celdas/s por unidad de potencia útil (se ajusta solo)

# Velocidad real aproximada (para estimar tiempos al repartir cubos).
# El panel la mide (botón "Ir recto y medir"). No tiene que ser exacta.
CELDAS_POR_S = 7.0         # a VEL_CRUCERO (medido en casa: ~8 celdas/s con 0,2-0,3)
GRADOS_POR_S = 90.0        # a GIRO_MAX

# ---------------------------------------------------------------------------
# Estrategia
# ---------------------------------------------------------------------------
PRE_AGARRE = 4.5           # el robot se para AGARRE+PRE_AGARRE antes del cubo
APROX_ZONA = 5.0           # entra a la zona en línea recta los últimos N celdas
MARGEN_BORDE = 4.0         # el centro del robot no planifica más afuera que esto
RETROCESO = 5.0            # cuánto retrocede después de soltar un cubo
RADIO_CUBO_OBST = 6.0      # radio para esquivar otros cubos
RADIO_ROVER_OBST = 9.5     # radio para esquivar al compañero en la ruta
TIEMPO_MAX_TAREA = 70.0    # si un cubo tarda más, se replanifica

# Anti-choque con el compañero (distancias entre centros, en celdas)
DIST_FRENO = 8.5           # frena siempre si el otro está adelante y así de cerca
DIST_CEDER = 13.0          # el de menor prioridad espera si el otro está adelante
TRAMO = 4.0                # PARE Y SIGA: celdas que anda (según él) antes de pararse a esperar una foto
TRAMO_LARGO = 10.0         # ídem lejos del borde, del compañero y de otros cubos
TRAMO_FINO = 1.5           # ídem empujando el cubo cerca de su zona (no pasarse)
TRAMO_MAX_S = 2.0          # y nunca más de esto andando sin pararse
ESPERA_MAX_PAR_MS = 5000   # lo máximo que un robot espera porque su camino cruza el del otro
TRAMO_GIRO_S = 2.0         # girando en el lugar (el giroscopio sabe cuánto giró)
FOTO_FRESCA_MS = 400       # cámara rápida (5 fotos en 1 s y atraso menor a esto): anda sin parar
MARGEN_FOTO_MS = 150       # la foto tiene que ser de al menos esto DESPUÉS de pararse
ESPERA_MAX = 2.5           # s esperando antes de retroceder para destrabar
CORREDOR = 11.0            # dos caminos a menos de esto "se cruzan" (no se usan a la vez)

# ---------------------------------------------------------------------------
# Confianza en los datos (CONTRATO 6.2 y 6.4)
# ---------------------------------------------------------------------------
EDAD_MAX_ROVER_MS = 500    # mi propia pose más vieja que esto: no navego con ella
SIN_DATOS_S = 0.6          # sin mensajes nuevos este tiempo: freno
EDAD_CUBO_FRESCO_MS = 300

# Arranque: el reglamento habla de READY, pero el sistema oficial de visión
# usa READY = "preparación de 1 minuto" y RUNNING = "ronda en juego" (el
# cronómetro oficial arranca en RUNNING). Por eso se arranca en RUNNING.
FASE_ARRANQUE = "RUNNING"
