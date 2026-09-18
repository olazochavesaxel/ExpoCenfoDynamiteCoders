# Prueba OPCIONAL del acelerometro/giroscopio - adaptado de code_acc.py
# del repo oficial del reto.
#
# Requiere la libreria adafruit_lsm6ds en la carpeta lib/ de CIRCUITPY.
# A diferencia del ejemplo original, esta version NO escribe un CSV a
# disco: por defecto, mientras el robot esta conectado por USB como
# unidad de almacenamiento, CircuitPython no puede escribir archivos
# desde code.py (da error de "Read-only filesystem") a menos que se
# configure un boot.py especial. Si mas adelante quieren registrar datos,
# avisen y lo armamos con ese ajuste.
#
# Para usar: copiar este archivo junto a ideaboard.py a la raiz de la
# unidad CIRCUITPY, renombrandolo a code.py.

from time import sleep
import board
from adafruit_lsm6ds import Rate, AccelRange, GyroRange
from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC

i2c = board.I2C()
sensor = LSM6DS3TRC(i2c, 0x6B)

sensor.accelerometer_range = AccelRange.RANGE_8G
sensor.gyro_range = GyroRange.RANGE_2000_DPS
sensor.accelerometer_data_rate = Rate.RATE_1_66K_HZ
sensor.gyro_data_rate = Rate.RATE_1_66K_HZ


def promedio_lecturas(muestras=5):
    sx = sy = sz = sgx = sgy = sgz = 0
    for _ in range(muestras):
        x, y, z = sensor.acceleration
        gx, gy, gz = sensor.gyro
        sx += x
        sy += y
        sz += z
        sgx += gx
        sgy += gy
        sgz += gz
    return (round(sx / muestras, 1), round(sy / muestras, 1), round(sz / muestras, 1),
            round(sgx / muestras, 1), round(sgy / muestras, 1), round(sgz / muestras, 1))


print("Leyendo IMU. Mueve y gira el robot para ver los valores cambiar.")
sleep(2)

while True:
    ax, ay, az, gx, gy, gz = promedio_lecturas()
    print("Acel: {:.1f}, {:.1f}, {:.1f}  |  Giro: {:.1f}, {:.1f}, {:.1f}".format(
        ax, ay, az, gx, gy, gz))
    sleep(0.2)
