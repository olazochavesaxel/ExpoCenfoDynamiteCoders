# config.py
#
# Configuracion central del rover para la logica de competencia.
# HAY QUE LLENAR/REVISAR ESTO ANTES DE PROBAR EN LA CANCHA REAL.

# --- Identidad de este rover ---
# Debe coincidir con el ID del marcador ArUco pegado a ESTE robot (10 u 11,
# segun el robot.md / aruco/README.md del repo oficial).
ROBOT_ID = 10

# --- WiFi de la cancha ---
# Llenar el dia de la prueba/competencia con la red real del lugar.
WIFI_SSID = "TU_RED_WIFI"
WIFI_PASSWORD = "TU_PASSWORD"

# --- Sistema de vision (CONTRATO.md v2) ---
# IP de la computadora que corre el sistema de vision, en la MISMA red que
# el robot. Se obtiene con ipconfig (Windows) en esa computadora. NUNCA usar
# 127.0.0.1 aqui (ver seccion "Conectarse desde el robot" del contrato).
VISION_HOST = "192.168.1.100"
VISION_PORT = 2026

# --- Comunicacion entre rovers (ESP-NOW) ---
# MAC del OTRO rover. Se imprime sola al arrancar main.py en ese robot
# ("Mi MAC: ..."). Copiarla aqui, y viceversa en el config.py del otro robot.
PEER_MAC = "AA:BB:CC:DD:EE:FF"

# --- Velocidades de motor (rango -1.0 a 1.0) ---
MOTOR_SPEED_CRUISE = 0.55   # avanzando hacia el punto de partida del empuje
MOTOR_SPEED_SLOW = 0.32     # empujando el cubo (mas lento = mas control)
MOTOR_SPEED_TURN = 0.30     # girando en el lugar

# --- Umbrales de control ---
ANGLE_TOLERANCE_DEG = 8         # margen para considerar "ya apunta bien"
DISTANCE_ARRIVED_CELLS = 1.0    # margen para considerar "ya llego"

# --- Sensor ultrasonico (para uso futuro: evasion de emergencia) ---
ULTRASONIC_STOP_CM = 8
