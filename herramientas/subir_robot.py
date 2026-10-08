# subir_robot.py — copia el programa a un CenfoBot por el CABLE USB.
#
# La IdeaBoard usa un ESP32 "clásico": NO aparece como unidad CIRCUITPY en el
# Explorador de archivos. Los archivos se pasan por el puerto serie, igual que
# hace Thonny con "Subir a /", pero todos de una vez y sin equivocarse.
#
# Una sola vez:   python -m pip install pyserial
#
# Uso (con Thonny CERRADO, porque ocupa el puerto):
#   python herramientas/subir_robot.py 10            programa de competencia al robot 10
#   python herramientas/subir_robot.py 11 --prueba   pruebas SIN cámara al robot 11
#   python herramientas/subir_robot.py --puertos     lista los puertos USB-serie
#
# Opciones:
#   --puerto COM5   si hay varios puertos y no adivina cuál es el robot
#   --limpiar       borra del robot los archivos de la versión anterior
#   --unidad E:     (solo si alguna vez el robot SÍ aparece como unidad)
#
# Copia a la raíz del robot: todos los .py de firmware/rover, robot_10.py (u
# 11) con el nombre robot.py, y secretos.py. code.py va al final; al terminar
# el robot se reinicia solo y arranca.

import argparse
import os
import shutil
import string
import sys
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(AQUI, "..", "firmware", "rover"))

NO_COPIAR = {"robot_10.py", "robot_11.py", "secretos_ejemplo.py", "secretos.py", "robot.py", "code.py"}
VIEJOS = ["main.py", "motion.py", "strategy.py", "coordination.py", "geometry.py",
          "vision_client.py", "config_ROBOT_10.py", "config_ROBOT_11.py"]
LIBS = ["adafruit_motor", "neopixel", "simpleio", "adafruit_lsm6ds", "adafruit_register",
        "adafruit_bus_device", "adafruit_pixelbuf"]


def lista_de_archivos(robot, prueba):
    sec = os.path.join(FW, "secretos.py")
    if not os.path.isfile(sec):
        sys.exit("Falta firmware/rover/secretos.py (copiá secretos_ejemplo.py con ese nombre y llenalo).")
    copias = [(os.path.join(FW, f), f) for f in sorted(os.listdir(FW))
              if f.endswith(".py") and f not in NO_COPIAR]
    copias.append((os.path.join(FW, "robot_%s.py" % robot), "robot.py"))
    copias.append((sec, "secretos.py"))
    principal = os.path.join(FW, "code.py")
    if prueba:
        principal = os.path.normpath(os.path.join(FW, "..", "pruebas", "prueba_sin_camara.py"))
    copias.append((principal, "code.py"))            # siempre el último
    return copias


# ---------------------------------------------------------------------------
# Por el puerto serie (raw REPL de CircuitPython)
# ---------------------------------------------------------------------------

class Placa:
    def __init__(self, puerto):
        import serial
        try:
            # DTR/RTS apagados: en muchas placas ESP32 esas líneas reinician la placa
            self.s = serial.Serial()
            self.s.port = puerto
            self.s.baudrate = 115200
            self.s.timeout = 0.2
            self.s.dtr = False
            self.s.rts = False
            self.s.open()
            time.sleep(1.0)
        except serial.SerialException as e:
            sys.exit("No pude abrir %s: %s\n¿Está Thonny abierto? Cerralo (ocupa el puerto) y probá de nuevo." % (puerto, e))

    def _leer_hasta(self, fin, segundos=10):
        datos = b""
        limite = time.time() + segundos
        while time.time() < limite:
            d = self.s.read(256)
            if d:
                datos += d
                if datos.endswith(fin):
                    return datos
        raise TimeoutError("la placa no respondió (esperaba %r, llegó %r)" % (fin, datos[-200:]))

    def entrar(self):
        # Varios intentos: Ctrl-C detiene el programa que esté corriendo, Enter
        # sale del "Press any key", Ctrl-A entra al modo "raw REPL".
        for intento in range(6):
            for _ in range(3):
                self.s.write(b"\r\x03")
                time.sleep(0.15)
                self.s.write(b"\x03")
                time.sleep(0.3)
            time.sleep(0.5 * intento)
            self.s.write(b"\r\x02\r")
            time.sleep(0.3)
            self.s.reset_input_buffer()
            self.s.write(b"\r\x01")
            try:
                self._leer_hasta(b"raw REPL; CTRL-B to exit\r\n>", 3)
                return
            except TimeoutError:
                print("   la placa no contesta todavía, reintento (%d/6)..." % (intento + 1))
        self.s.close()
        sys.exit(
            "\nLa placa no respondió por el USB. No se subió nada. Probá en este orden:\n"
            "  1) Apretá el botón de RESET (EN/RST) de la IdeaBoard, esperá 5 s y corré de nuevo.\n"
            "  2) Desenchufá el USB, esperá 5 s, enchufalo y corré de nuevo.\n"
            "  3) Abrí Thonny: si tampoco muestra '>>>', apretá Stop/Restart ahí, cerrá Thonny y reintentá.\n"
            "  4) Probá otro puerto USB de la compu u otro cable.")

    def ejecutar(self, codigo, segundos=10):
        datos = codigo.encode("utf-8")
        for i in range(0, len(datos), 128):          # de a poquito: el ESP32 tiene poco buffer
            self.s.write(datos[i:i + 128])
            self.s.flush()
            time.sleep(0.01)
        self.s.write(b"\x04")
        r = self._leer_hasta(b"\x04>", segundos)
        if not r.startswith(b"OK"):
            raise RuntimeError("respuesta rara de la placa: %r" % r[:200])
        salida, _, error = r[2:-2].partition(b"\x04")
        if error.strip():
            raise RuntimeError(error.decode("utf-8", "replace"))
        return salida.decode("utf-8", "replace")

    def escribir_archivo(self, origen, nombre):
        datos = open(origen, "rb").read()
        self.ejecutar("f=open(%r,'wb')" % ("/" + nombre))
        for i in range(0, len(datos), 768):
            self.ejecutar("f.write(%r)" % datos[i:i + 768])
        self.ejecutar("f.close()")
        tam = self.ejecutar("import os\nprint(os.stat(%r)[6])" % ("/" + nombre)).strip()
        if tam != str(len(datos)):
            raise RuntimeError("%s quedó con %s bytes en vez de %d" % (nombre, tam, len(datos)))

    def reiniciar(self):
        self.s.write(b"\x02")                        # salir del raw REPL
        time.sleep(0.2)
        self.s.write(b"\x04")                        # Ctrl-D: reinicio suave -> corre code.py
        time.sleep(0.3)
        self.s.close()


def puertos():
    from serial.tools import list_ports
    return list(list_ports.comports())


def elegir_puerto():
    ps = puertos()
    # chips USB-serie típicos de las placas ESP32 (CP210x, CH340, FTDI)
    buenos = [p for p in ps if p.vid in (0x10C4, 0x1A86, 0x0403, 0x303A)]
    cand = buenos or ps
    if len(cand) == 1:
        return cand[0].device
    if not cand:
        sys.exit("No veo ningún puerto USB-serie. ¿Está conectado el robot? ¿El cable pasa datos?")
    print("Hay varios puertos; elegí uno con --puerto:")
    for p in cand:
        print("  ", p.device, "-", p.description)
    sys.exit(1)


def subir_serie(a, copias):
    puerto = a.puerto or elegir_puerto()
    print("Robot en el puerto", puerto, "(si Thonny está abierto, cerralo)")
    p = Placa(puerto)
    p.entrar()
    # el programa del robot usa un "perro guardián" que reinicia la placa si se
    # traba; acá se apaga para que no la reinicie en medio de la carga
    p.ejecutar("try:\n import microcontroller\n microcontroller.watchdog.deinit()\nexcept Exception:\n pass")
    print("  ", p.ejecutar("print(open('/boot_out.txt').readline().strip())").strip())
    # se prueba IMPORTAR cada librería: algunas vienen incluidas en CircuitPython
    # (no están en lib/ pero existen igual)
    faltan = []
    for l in LIBS:
        r = p.ejecutar("try:\n import %s\n print('si')\nexcept ImportError:\n print('no')" % l)
        if "no" in r.split():
            faltan.append(l)
    if faltan:
        print("AVISO: el robot no tiene estas librerías:", ", ".join(faltan))
    if a.limpiar:
        for v in VIEJOS:
            r = p.ejecutar("import os\ntry:\n os.remove(%r)\n print('borrado')\nexcept OSError:\n pass" % ("/" + v))
            if "borrado" in r:
                print("  borrado (versión vieja):", v)
    for n, (origen, nombre) in enumerate(copias, 1):
        print("  [%2d/%d] %s" % (n, len(copias), nombre), end="", flush=True)
        t = time.time()
        p.escribir_archivo(origen, nombre)
        print("  ok (%.1f s)" % (time.time() - t))
    p.reiniciar()
    print()
    print("Listo. El robot se reinició y está corriendo el programa nuevo.")
    print("Abrí Thonny para ver lo que dice (si no muestra nada, clic en la consola y Ctrl+D).")


# ---------------------------------------------------------------------------
# Como unidad de disco (por si alguna placa sí aparece como CIRCUITPY)
# ---------------------------------------------------------------------------

def subir_unidad(destino, a, copias):
    if a.limpiar:
        for v in VIEJOS:
            if os.path.isfile(os.path.join(destino, v)):
                os.remove(os.path.join(destino, v))
    for origen, nombre in copias:
        shutil.copyfile(origen, os.path.join(destino, nombre))
        print("  copiado", nombre)
    print("Listo.")


def buscar_unidad():
    if os.name == "nt":
        for l in string.ascii_uppercase[3:]:
            if os.path.isfile("%s:\\boot_out.txt" % l):
                return "%s:\\" % l
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("robot", nargs="?", choices=["10", "11"], help="ID del marcador ArUco de ESTE robot")
    ap.add_argument("--puerto")
    ap.add_argument("--unidad")
    ap.add_argument("--limpiar", action="store_true")
    ap.add_argument("--prueba", action="store_true", help="sube las pruebas SIN cámara como code.py")
    ap.add_argument("--puertos", action="store_true", help="lista los puertos y sale")
    a = ap.parse_args()
    try:
        import serial  # noqa: F401
    except ImportError:
        if not a.unidad:
            sys.exit("Falta pyserial. Instalalo una vez con:  python -m pip install pyserial")
    if a.puertos:
        for p in puertos():
            print(p.device, "-", p.description)
        return
    if not a.robot:
        ap.error("decime qué robot: 10 u 11")
    copias = lista_de_archivos(a.robot, a.prueba)
    destino = a.unidad or buscar_unidad()
    if destino:
        subir_unidad(destino, a, copias)
    else:
        subir_serie(a, copias)
    print("RECORDÁ: al encender, dejalo QUIETO 2 s (luz roja) mientras mide el giroscopio.")


if __name__ == "__main__":
    main()
