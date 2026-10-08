# vision_client.py
#
# Cliente TCP del contrato de telemetria del Vision Rover Challenge (v2,
# ver vision-system/contrato/CONTRATO.md del repo oficial). Se conecta al
# sistema de vision, acumula el buffer y entrega el ULTIMO mensaje JSON
# completo recibido en cada poll(). Reconecta si la conexion se cae.
#
# La vision NUNCA espera nada de nosotros: solo escribe. No hay que mandarle
# nada, ni "suscribirse".

import json
import time
import wifi
import socketpool


def connect_wifi(ssid, password, retries=10):
    """Conecta la radio WiFi (modo estacion) a la red de la cancha."""
    if wifi.radio.connected:
        return

    print("Conectando a WiFi:", ssid)
    attempt = 0
    while True:
        try:
            wifi.radio.connect(ssid, password)
            break
        except ConnectionError as error:
            attempt += 1
            print("Fallo WiFi intento", attempt, ":", error)
            if attempt >= retries:
                raise
            time.sleep(1)

    print("WiFi conectado. IP:", wifi.radio.ipv4_address)


class VisionClient:

    def __init__(self, host, port, pool=None):
        self.host = host
        self.port = port
        self.pool = pool or socketpool.SocketPool(wifi.radio)
        self.socket = None
        self.buffer = b""
        self.last_message = None
        self.connected = False

    def connect(self):
        print("Conectando al sistema de vision", self.host, self.port)
        sock = self.pool.socket(self.pool.AF_INET, self.pool.SOCK_STREAM)
        sock.connect((self.host, self.port))
        sock.setblocking(False)
        self.socket = sock
        self.buffer = b""
        self.connected = True
        print("Conectado al sistema de vision")

    def close(self):
        if self.socket is not None:
            try:
                self.socket.close()
            except OSError:
                pass
        self.socket = None
        self.connected = False

    def ensure_connected(self):
        """Reconecta si hace falta. Llamar seguido desde el loop principal."""
        if self.connected:
            return True
        try:
            self.connect()
            return True
        except OSError as error:
            print("No se pudo conectar al sistema de vision:", error)
            return False

    def poll(self):
        """
        Lee lo que haya disponible sin bloquear y devuelve el ULTIMO mensaje
        JSON completo recibido en esta llamada (o None si no llego ninguno
        nuevo). Si la conexion se cae la marca como desconectada: quien
        llama debe usar ensure_connected() para reintentar.
        """
        if not self.connected:
            return None

        try:
            chunk = self.socket.recv(4096)
        except OSError:
            # No hay datos disponibles ahora mismo (normal, no bloqueante).
            return self._latest_from_buffer()

        if chunk == b"":
            print("El sistema de vision cerro la conexion")
            self.connected = False
            return self._latest_from_buffer()

        if chunk:
            self.buffer += chunk

        return self._latest_from_buffer()

    def _latest_from_buffer(self):
        latest = None
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            if not line:
                continue
            try:
                latest = json.loads(line)
            except ValueError:
                continue  # linea incompleta o corrupta: se descarta

        if latest is not None:
            self.last_message = latest

        return latest
