# robot_10.py — datos del robot que lleva el marcador ArUco 10.
# herramientas/subir_robot.py lo copia al robot con el nombre robot.py.
# Calibrado con firmware/pruebas/prueba_sin_camara.py el 2-oct-2026:
#   ruedas OK, giro 90° -> 91,9° (giroscopio OK), recto (3-oct, sin tocarlo): se tuerce 18° a la izquierda en 2 s -> GANANCIA_DER 0.928.

ROBOT_ID = 10
MOTOR_IZQ_SIGNO = 1.0      # -1.0 si con "avanzar" la rueda izquierda va para atrás
MOTOR_DER_SIGNO = 1.0      # -1.0 si con "avanzar" la rueda derecha va para atrás
GANANCIA_DER = 0.928        # >1 si el robot se tuerce a la derecha, <1 si se tuerce a la izquierda
GIRO_SIGNO = 1.0           # el robot lo verifica solo con la cámara; si lo corrige, avisa en el panel
GIRO_ESCALA = 1.0
MOTOR_MINIMO = 0.16        # potencia mínima con la que el robot empieza a moverse
