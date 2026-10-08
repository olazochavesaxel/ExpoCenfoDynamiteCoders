# coordination.py
#
# Coordinacion minima entre los dos rovers via ESP-NOW: cada uno anuncia
# que color de cubo eligio y donde esta, para evitar que los dos vayan por
# el mismo cubo. Basado en el ejemplo oficial codigos/espnow_bidirectional.py.
#
# DIFERENCIA IMPORTANTE con ese ejemplo: nosotros SI necesitamos estar
# conectados por WiFi normal (STA) al mismo tiempo, para hablar con el
# sistema de vision. La radio del ESP32 es una sola, asi que ESP-NOW tiene
# que usar el MISMO canal en el que ya quedamos conectados por WiFi -- por
# eso aqui NO se hace el truco de wifi.radio.start_ap()/stop_ap() del
# ejemplo (ese truco es solo para fijar un canal cuando NO hay conexion
# WiFi normal). En vez de eso, se lee el canal real con
# current_wifi_channel() despues de conectar el WiFi (ver main.py).
#
# Formato del mensaje (texto simple separado por comas, entra holgado en
# los ~250 bytes que permite ESP-NOW):
#   "<robot_id>,<color_o_NADA>,<col>,<row>,<seq>"

import time
import wifi
import espnow


def mac_from_string(mac_string):
    return bytes(int(part, 16) for part in mac_string.split(":"))


def mac_to_string(mac):
    return ":".join("{:02X}".format(byte) for byte in mac)


def current_wifi_channel():
    """
    Canal en el que quedo la radio tras conectarse por WiFi. VERIFICAR EN
    LA CANCHA: si los dos rovers estan conectados al WiFi de la vision pero
    no se reciben entre si por ESP-NOW, revisar esto primero.
    """
    return wifi.radio.ap_info.channel


class Coordinator:

    def __init__(self, robot_id, peer_mac, channel):
        self.robot_id = robot_id
        self.channel = channel
        self.peer_state = None
        self.peer_last_seen = 0
        self._seq = 0

        print("Mi MAC:", mac_to_string(wifi.radio.mac_address))
        print("Canal ESP-NOW (= canal WiFi actual):", channel)

        self.esp = espnow.ESPNow()
        peer = espnow.Peer(mac=mac_from_string(peer_mac), channel=channel)
        self.esp.peers.append(peer)

    def anunciar(self, cubo_color, rover):
        self._seq += 1
        color = cubo_color if cubo_color else "NADA"
        mensaje = "{},{},{:.2f},{:.2f},{}".format(
            self.robot_id, color, rover["col"], rover["row"], self._seq
        )
        try:
            self.esp.send(mensaje.encode("utf-8"))
        except Exception as error:
            print("Error enviando ESP-NOW:", error)

    def recibir_pendientes(self):
        """Procesa todos los mensajes ESP-NOW pendientes, sin bloquear."""
        while True:
            packet = self.esp.read()
            if packet is None:
                break
            try:
                texto = packet.msg.decode("utf-8")
                partes = texto.split(",")
                robot_id = int(partes[0])
                color = None if partes[1] == "NADA" else partes[1]
                col = float(partes[2])
                row = float(partes[3])
                seq = int(partes[4])
            except (ValueError, IndexError):
                continue

            if robot_id == self.robot_id:
                continue

            self.peer_state = {
                "robot_id": robot_id,
                "cubo_color": color,
                "col": col,
                "row": row,
                "seq": seq,
            }
            self.peer_last_seen = time.monotonic()

    def peer_esta_activo(self, timeout=3.0):
        return (
            self.peer_state is not None
            and (time.monotonic() - self.peer_last_seen) < timeout
        )

    def peer_eligio(self):
        """Color que eligio el otro rover, o None si no se sabe / no eligio."""
        if not self.peer_esta_activo():
            return None
        return self.peer_state["cubo_color"]
