# red.py — WiFi, cliente de la telemetría de la cámara (CONTRATO v3) y el
# canal OPCIONAL con el panel de la compu (solo para probar en casa).
# Específico de CircuitPython (módulos wifi y socketpool).

import time
import wifi

EAGAIN = 11
ETIMEDOUT = 116


def conectar_wifi(ssid, clave, hw=None, intentos=0, alimentar=None):
    """Conecta al WiFi. intentos=0 => insiste para siempre (titilando en naranja).
    alimentar: función que se llama en cada intento (el perro guardián)."""
    # ssid puede ser una LISTA de redes [(nombre, clave), ...]: se prueban en orden
    redes = ssid if isinstance(ssid, (list, tuple)) and ssid and isinstance(ssid[0], (list, tuple)) \
        else [(ssid, clave)]
    n = 0
    while not wifi.radio.connected:
        n += 1
        ssid, clave = redes[(n - 1) % len(redes)]
        if alimentar:
            alimentar()
        if hw:
            hw.led((80, 30, 0) if n % 2 else (0, 0, 0))
        try:
            print("Conectando a WiFi '{}' (intento {})...".format(ssid, n))
            try:
                wifi.radio.connect(ssid, clave, timeout=5)
            except TypeError:                     # CircuitPython viejo: sin timeout
                wifi.radio.connect(ssid, clave)
        except Exception as e:  # noqa
            print("  no se pudo:", e)
            if intentos and n >= intentos * len(redes):
                return False
            time.sleep(1.0)
    sin_ahorro()
    print("WiFi OK. Mi IP:", wifi.radio.ipv4_address,
          " canal:", _canal(), " senal:", senal(), "dBm")
    return True


def sin_ahorro():
    """Apaga el ahorro de energía del WiFi: con el ahorro prendido la radio
    'duerme' entre mensajes, los datos de la cámara llegan a los saltos y la
    radio con el compañero (ESP-NOW) pierde mensajes."""
    try:
        wifi.radio.power_management = wifi.PowerManagement.NONE
    except Exception:  # noqa — versiones de CircuitPython sin esta opción
        pass


def senal():
    """Intensidad de la señal del WiFi en dBm (-50 muy buena, -80 mala), o None."""
    try:
        return wifi.radio.ap_info.rssi
    except Exception:  # noqa
        return None


def _canal():
    try:
        return wifi.radio.ap_info.channel
    except Exception:  # noqa
        return "?"


def mi_mac():
    return ":".join("{:02X}".format(b) for b in wifi.radio.mac_address)


class ClienteVision:
    """Lee la telemetría NDJSON sin bloquear y entrega SOLO la línea más nueva.

    Reglas del contrato que cumple:
      - TCP no respeta los límites de los mensajes: se acumula y se corta por '\\n'
      - "quédense con el último mensaje": se vacía TODO lo que llegó y se
        descarta lo viejo, así nunca navegamos con una cola atrasada
      - si se cae la conexión, se reconecta (la visión manda el presente)
    """

    def __init__(self, pool, host, port):
        self.pool = pool
        self.host = host
        self.port = port
        self.sock = None
        self.buf = bytearray(1024)
        self.pend = b""
        self.conectado = False
        self._ult_intento = -10.0
        self.recibidas = 0
        self.fallos = 0           # intentos de conexión fallidos seguidos
        self.cortes = 0           # cuántas veces se cortó (para el panel)
        self.error = ""           # el último error (para el panel)

    def conectar(self):
        if time.monotonic() - self._ult_intento < 1.0:
            return False
        self._ult_intento = time.monotonic()
        self.cerrar()
        s = None
        try:
            s = self.pool.socket(self.pool.AF_INET, self.pool.SOCK_STREAM)
            s.settimeout(2)
            s.connect((self.host, self.port))
            s.setblocking(False)
            self.sock = s
            self.pend = b""
            self.conectado = True
            self.fallos = 0
            print("Conectado a la vision en {}:{}".format(self.host, self.port))
            return True
        except Exception as e:  # noqa
            print("No conecta a la vision {}:{} -> {}".format(self.host, self.port, e))
            self.error = "conectar: {}".format(e)[:60]
            self.fallos += 1
            # IMPORTANTE: cerrar el socket que falló. Antes quedaba abierto, y
            # como el ESP32 tiene pocos sockets, después de unos intentos ya no
            # podía conectarse nunca más (había que apagar y prender el robot).
            if s is not None:
                try:
                    s.close()
                except Exception:  # noqa
                    pass
            self.cerrar()
            return False

    def cerrar(self, motivo=None):
        if motivo:
            self.cortes += 1
            self.error = motivo[:60]
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:  # noqa
                pass
        self.sock = None
        self.conectado = False

    def leer(self):
        """Devuelve la última línea completa (bytes) o None si no llegó nada nuevo.
        Lee de a pedazos chicos (2 KB) y guarda solo la última línea completa:
        así nunca pide un bloque grande de memoria aunque se haya atrasado."""
        if not self.conectado:
            return None
        ultima = None
        for _ in range(8):
            try:
                n = self.sock.recv_into(self.buf)
            except OSError as e:
                cod = e.args[0] if e.args else None
                if cod in (EAGAIN, ETIMEDOUT):
                    break
                print("Se corto la conexion con la vision:", e)
                self.cerrar("recv: {}".format(e))
                return None
            if n == 0:
                print("La vision cerro la conexion")
                self.cerrar("la vision cerro")
                return None
            d = bytes(self.buf[:n])
            fin = d.rfind(b"\n")
            if fin < 0:
                self.pend = self.pend + d if len(self.pend) < 3000 else b""
                continue
            ini = d.rfind(b"\n", 0, fin)
            if ini >= 0:
                ultima = d[ini + 1:fin]
            else:
                ultima = self.pend + d[:fin]
            self.pend = d[fin + 1:]
        if ultima is not None:
            self.recibidas += 1
        return ultima


class CanalPanel:
    """Canal UDP con herramientas/panel.py (opcional).

    - Si DEPURACION_HOST tiene una IP, el robot le manda su estado 5 veces
      por segundo (solo informa: no recibe decisiones).
    - Si MODO_PRUEBAS = True, además escucha órdenes de prueba. En la
      competencia MODO_PRUEBAS debe ser False: el robot no escucha nada.
    """

    def __init__(self, pool, cfg):
        self.cfg = cfg
        self.dest = (cfg.DEPURACION_HOST, cfg.DEPURACION_PUERTO) if cfg.DEPURACION_HOST else None
        self.sock = None
        self.buf = bytearray(1100)
        self._ult = 0
        self.origen = None        # IP de la compu del panel (por su baliza)
        self.tele = None          # telemetría que reenvía el panel por UDP (solo pruebas)
        if self.dest is None and not cfg.MODO_PRUEBAS and not getattr(cfg, "TELE_UDP", False):
            return
        try:
            s = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
            s.bind(("0.0.0.0", cfg.PRUEBAS_PUERTO))
            s.setblocking(False)
            self.sock = s
            print("Canal con el panel listo (puerto {}), modo pruebas: {}".format(
                cfg.PRUEBAS_PUERTO, cfg.MODO_PRUEBAS))
        except Exception as e:  # noqa
            print("Sin canal con el panel:", e)

    def leer_orden(self):
        """Lee lo que mandó el panel. Devuelve una orden de prueba (texto) o None.
        Si llega telemetría reenviada por el panel, la deja en self.tele."""
        if self.sock is None:
            return None
        pruebas = self.cfg.MODO_PRUEBAS
        # TELE_UDP: solo acepta la COPIA de la telemetría oficial (nunca órdenes)
        tele_ok = pruebas or getattr(self.cfg, "TELE_UDP", False)
        orden = None
        for _ in range(4):
            try:
                n, origen = self.sock.recvfrom_into(self.buf)
            except OSError:
                break
            if n <= 0:
                break
            if self.buf[0] == 123:            # '{': telemetría (el panel la reenvía)
                if tele_ok:
                    self.tele = bytes(self.buf[:n]).strip()
                continue
            if not pruebas:
                continue                      # sin modo pruebas: no se escucha nada más
            if self.dest is None:
                self.dest = (origen[0], self.cfg.DEPURACION_PUERTO)    # le contesto a quien me habla
            try:
                txt = bytes(self.buf[:n]).decode("utf-8")
            except UnicodeError:
                continue
            if txt.startswith("BALIZA"):
                # el panel avisa "acá estoy" cada segundo: es la compu de la visión.
                self.origen = origen[0]
                self.dest = (origen[0], self.cfg.DEPURACION_PUERTO)
                continue
            if orden is None:
                orden = txt
        return orden

    def informar(self, ahora_ms, texto_fn):
        if self.sock is None or self.dest is None or ahora_ms - self._ult < 200:
            return
        self._ult = ahora_ms
        try:
            self.sock.sendto(texto_fn().encode("utf-8"), self.dest)
        except Exception:  # noqa
            pass
