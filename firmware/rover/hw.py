# hw.py — el ÚNICO archivo que toca el hardware del CenfoBot (CircuitPython).
#
#   - motores (con los signos y la ganancia de calibración de robot.py)
#   - giroscopio de la IMU LSM6DS3TRC (dirección I2C 0x6B), en grados/s
#   - luz NeoPixel
#
# Se crea UNA sola vez en code.py. Crear IdeaBoard() dos veces es lo que
# producía el error "IO2 in use" en la versión anterior.

import time
import board

RAD_A_GRAD = 57.29578


class Hardware:

    def __init__(self, cfg):
        self.cfg = cfg
        self._led = None
        self._np = None
        self.imu = None
        self.deriva = 0.0          # rad/s que marca el giroscopio estando quieto
        try:
            from ideaboard import IdeaBoard
            self.ib = IdeaBoard()
            self.m1 = self.ib.motor_1
            self.m2 = self.ib.motor_2
            self._np = self.ib
        except ValueError as e:
            # Algo ya usa algún pin (normalmente: el programa se recargó sin
            # reiniciar). Armamos los motores a mano, igual que ideaboard.py.
            print("AVISO IdeaBoard:", e, "-> motores sin la libreria (sin luz)")
            import pwmio
            from adafruit_motor import motor
            self.ib = None
            self.m1 = motor.DCMotor(pwmio.PWMOut(board.IO12, frequency=50),
                                    pwmio.PWMOut(board.IO14, frequency=50))
            self.m2 = motor.DCMotor(pwmio.PWMOut(board.IO13, frequency=50),
                                    pwmio.PWMOut(board.IO15, frequency=50))
        self.motores(0.0, 0.0)
        try:
            from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC
            self.imu = LSM6DS3TRC(board.I2C(), 0x6B)
            try:
                from adafruit_lsm6ds import Rate, GyroRange
                self.imu.gyro_range = GyroRange.RANGE_500_DPS   # más resolución que 2000
                self.imu.gyro_data_rate = Rate.RATE_416_HZ
            except Exception as e:  # noqa
                print("IMU con configuracion por defecto:", e)
        except Exception as e:  # noqa
            print("AVISO: no encontre la IMU (", e, "). Sigo solo con la camara.")
            self.imu = None

    # ------------------------------------------------------------- motores
    def motores(self, izq, der):
        c = self.cfg
        i = max(-1.0, min(1.0, izq * c.MOTOR_IZQ_SIGNO))
        d = max(-1.0, min(1.0, der * c.GANANCIA_DER * c.MOTOR_DER_SIGNO))
        self.m1.throttle = i      # motor_1 = rueda izquierda (test_motores.py)
        self.m2.throttle = d

    def parar(self):
        self.m1.throttle = 0
        self.m2.throttle = 0

    # --------------------------------------------------------- giroscopio
    def calibrar_giro(self, segundos=2.0):
        """Mide la deriva del giroscopio. EL ROBOT TIENE QUE ESTAR QUIETO."""
        if self.imu is None:
            return
        suma = 0.0
        n = 0
        fin = time.monotonic() + segundos
        while time.monotonic() < fin:
            try:
                z = self.imu.gyro[2]
            except OSError:
                continue
            if abs(z) < 0.05:          # descarta golpes
                suma += z
                n += 1
            time.sleep(0.005)
        if n:
            self.deriva = suma / n
        print("Deriva del giroscopio: {:.5f} rad/s ({} muestras)".format(self.deriva, n))

    def giro_dps(self):
        """Velocidad de giro en grados/s, + = antihorario (como theta). None si no hay IMU."""
        if self.imu is None:
            return None
        try:
            z = self.imu.gyro[2] - self.deriva
        except OSError:
            return None
        return z * RAD_A_GRAD * self.cfg.GIRO_SIGNO * self.cfg.GIRO_ESCALA

    # ---------------------------------------------------------------- luz
    def led(self, rgb):
        if rgb == self._led or self._np is None:
            return
        self._led = rgb
        try:
            self._np.pixel = rgb
        except Exception:  # noqa
            pass
