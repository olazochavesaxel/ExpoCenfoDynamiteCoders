# robot_11.py — datos del robot que lleva el marcador ArUco 11.
# herramientas/subir_robot.py lo copia al robot con el nombre robot.py.
# Prueba sin cámara del 3-oct-2026: giroscopio OK (+117° girando a mano a la izquierda),
# pero con "adelante" fue para ATRÁS y al pedir giro a la izquierda giró a la derecha
# -> los dos motores están cableados al revés: se invierten acá (MOTOR_*_SIGNO = -1).

ROBOT_ID = 11
MOTOR_IZQ_SIGNO = -1.0     # -1.0 si con "avanzar" la rueda izquierda va para atrás
MOTOR_DER_SIGNO = -1.0     # -1.0 si con "avanzar" la rueda derecha va para atrás
GANANCIA_DER = 1.0         # >1 si el robot se tuerce a la derecha, <1 si se tuerce a la izquierda
GIRO_SIGNO = 1.0           # el robot lo verifica solo con la cámara; si lo corrige, avisa en el panel
GIRO_ESCALA = 1.0
MOTOR_MINIMO = 0.15        # potencia mínima con la que el robot empieza a moverse
