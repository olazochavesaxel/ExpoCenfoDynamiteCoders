# secretos_ejemplo.py — COPIAR como secretos.py y llenar. secretos.py NO se
# sube a GitHub (está en .gitignore) porque tiene la clave del WiFi.

# OJO: el ESP32 solo usa WiFi de 2,4 GHz. Una red "5G" no le sirve.
WIFI_SSID = "NOMBRE_DE_LA_RED"
WIFI_PASSWORD = "CLAVE"
# (opcional) varias redes, se prueban en orden: [("red1", "clave1"), ("red2", "clave2")]
# WIFI_REDES = [("Mi celular", "clave"), ("Router de casa", "clave")]

# IP de la computadora donde corre la visión oficial (o el simulador).
# En esa compu: ipconfig -> "Dirección IPv4". NUNCA 127.0.0.1.
VISION_HOST = "192.168.1.100"
VISION_PORT = 2026

# IP de la compu con herramientas/panel.py para ver el estado del robot.
# None = no manda nada (para la competencia).
DEPURACION_HOST = None

# Solo para PROBAR EN CASA con el panel (herramientas/panel.py):
# MODO_PRUEBAS = True        # el panel puede darle órdenes de prueba
# En la competencia: dejar estas líneas comentadas (MODO_PRUEBAS queda en False).
