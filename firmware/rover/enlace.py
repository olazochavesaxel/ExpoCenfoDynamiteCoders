# enlace.py — radio entre los dos rovers (reglamento 7.2: "mecanismos
# inalámbricos disponibles en el hardware oficial").
#
# ESP-NOW por DIFUSIÓN (a FF:FF:FF:FF:FF:FF): no hace falta copiar la MAC del
# compañero. Cada mensaje lleva la etiqueta del equipo (config.ENLACE_EQUIPO)
# y el ID del robot, así se ignoran los mensajes propios y los de otros equipos.
#
# OJO con el canal: la radio del ESP32 es una sola. Como el robot está
# conectado al WiFi de la cancha, ESP-NOW usa EL MISMO canal que ese WiFi
# (channel=0 = "el canal actual"). Los dos rovers están en el mismo WiFi, así
# que coinciden solos.
#
# Si la radio falla, los rovers igual funcionan: cada uno ve al otro por la
# cámara y calcula el mismo reparto (solo pierden un poco de coordinación).


class EnlaceEspNow:
    def __init__(self):
        import espnow
        self.e = espnow.ESPNow()
        self.peer = espnow.Peer(mac=b"\xff\xff\xff\xff\xff\xff", channel=0)
        self.e.peers.append(self.peer)
        self.errores = 0

    def enviar(self, txt):
        try:
            self.e.send(txt.encode("utf-8"), self.peer)
        except Exception:  # noqa
            self.errores += 1

    def recibir(self):
        res = []
        for _ in range(8):
            try:
                p = self.e.read()
            except Exception:  # noqa
                break
            if p is None:
                break
            try:
                res.append(bytes(p.msg).decode("utf-8"))
            except UnicodeError:
                pass
        return res


class EnlaceUdp:
    """Alternativa por el WiFi (difusión UDP en la red local)."""

    def __init__(self, pool, puerto):
        import wifi
        self.puerto = puerto
        ip = [int(x) for x in str(wifi.radio.ipv4_address).split(".")]
        mask = [int(x) for x in str(wifi.radio.ipv4_subnet).split(".")]
        self.difusion = ".".join(str(ip[i] | (255 - mask[i])) for i in range(4))
        self.s = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
        self.s.bind(("0.0.0.0", puerto))
        self.s.setblocking(False)
        self.buf = bytearray(300)

    def enviar(self, txt):
        try:
            self.s.sendto(txt.encode("utf-8"), (self.difusion, self.puerto))
        except Exception:  # noqa
            pass

    def recibir(self):
        res = []
        for _ in range(8):
            try:
                n, _o = self.s.recvfrom_into(self.buf)
            except OSError:
                break
            try:
                res.append(bytes(self.buf[:n]).decode("utf-8"))
            except UnicodeError:
                pass
        return res


def crear(cfg, pool):
    try:
        if cfg.ENLACE == "udp":
            e = EnlaceUdp(pool, cfg.ENLACE_UDP_PUERTO)
        else:
            e = EnlaceEspNow()
        print("Radio entre rovers lista:", cfg.ENLACE)
        return e
    except Exception as ex:  # noqa
        print("AVISO: sin radio entre rovers ({}). Sigo igual, coordinando por camara.".format(ex))
        return None
