# config_ROBOT_10.py
#
# Copia de config.py YA LLENA para el robot que lleva el marcador ArUco 10.
#
# COMO USAR: copia este archivo a la unidad CIRCUITPY de ESE robot y,
# una vez ahi, renombralo a "config.py" (reemplazando el que ya hubiera).

# --- Identidad de este rover ---
ROBOT_ID = 10

# --- WiFi de la cancha ---
WIFI_SSID = "Protec 5.G"
WIFI_PASSWORD = "(se movio a firmware/rover/secretos.py)"

# --- Sistema de vision (CONTRATO.md v2) ---
# IP de la laptop que corre mock_publisher.py / el sistema de vision.
VISION_HOST = "Poner ip"
VISION_PORT = 2026

# --- Comunicacion entre rovers (ESP-NOW) ---
# TODAVIA SIN LLENAR: es un valor de relleno con formato valido, no la MAC
# real del robot 11. No rompe nada (el robot arranca igual), pero mientras
# este asi, este robot no va a "escuchar" al otro por ESP-NOW. Cuando
# tengas los dos robots y corras main.py/code.py en el robot 11, copia de
# su consola la linea "Mi MAC: ..." y pegala aqui.
PEER_MAC = "AA:BB:CC:DD:EE:F1"

# --- Velocidades de motor (rango -1.0 a 1.0) ---
MOTOR_SPEED_CRUISE = 0.55   # avanzando hacia el punto de partida del empuje
MOTOR_SPEED_SLOW = 0.32     # empujando el cubo (mas lento = mas control)
MOTOR_SPEED_TURN = 0.30     # girando en el lugar

# --- Umbrales de control ---
ANGLE_TOLERANCE_DEG = 8         # margen para considerar "ya apunta bien"
DISTANCE_ARRIVED_CELLS = 1.0    # margen para considerar "ya llego"

# --- Sensor ultrasonico (para uso futuro: evasion de emergencia) ---
ULTRASONIC_STOP_CM = 8
