# Guía del equipo DynamiteCoders: de cero a la competencia

Esta guía explica qué hace el código, cómo probarlo **sin robots** (con el simulador), cómo
probarlo **con los robots y la cámara**, y qué revisar el día de la competencia.

---

## 1. Qué tiene que pasar en la competencia (en criollo)

```
 cámara cenital ──► PC de la organización (sistema de visión OFICIAL, no se toca)
                         │  publica por WiFi, 20 veces por segundo, un JSON con:
                         │  dónde está cada robot, cada cubo, las zonas, la fase y el reloj
                         ▼
        ┌────────── Rover 10 ◄── ESP-NOW (radio) ──► Rover 11 ──────────┐
        │  lee el JSON, decide qué cubo le toca, planifica la ruta,      │
        │  mueve los motores, se coordina con el compañero               │
        └────────────────────────────────────────────────────────────────┘
```

* La PC **solo informa**. Toda decisión la toman los robots (reglamento 6.3 y 11).
* Fases del sistema oficial: `IDLE` → `READY` (1 minuto de preparación, el reloj cuenta para
  atrás) → `RUNNING` (arranca el cronómetro oficial) → `FINISHED`.
  **Nuestros robots arrancan solos al ver `RUNNING`.** Nadie toca nada después.
  > El reglamento usa la palabra "READY" para el arranque, pero en el sistema oficial el
  > cronómetro empieza en `RUNNING` (READY es la cuenta regresiva). Conviene **confirmarlo
  > con la organización**; si dicen que es en READY, se cambia una línea: `FASE_ARRANQUE`
  > en `config.py`.
* Gana quien deja **los 3 cubos completamente dentro de su zona** en menos tiempo. El campo
  `in_depot` de la telemetría es el veredicto oficial.
* **Regla 12.2.13:** para que los 3 cubos cuenten, **cada robot tiene que haber llevado al
  menos uno**. El reparto de cubos de nuestro código lo garantiza.

## 2. Qué hay en el repo

```
firmware/rover/        → lo que va ADENTRO de cada robot (CircuitPython)
  code.py              programa principal (arranca solo al encender)
  config.py            TODOS los números ajustables (velocidades, distancias...)
  robot_10.py / _11.py identidad y calibración de cada robot (se sube como robot.py)
  secretos.py          WiFi e IP de la visión (NO se sube a GitHub)
  cerebro.py           máquina de estados: elegir cubo → ir → agarrar → llevar → soltar
  coordinacion.py      radio con el compañero, reparto de cubos, ceder el paso
  movimiento.py        girar, avanzar, frenar, no chocar
  plan.py              reparto óptimo de cubos y rutas que esquivan obstáculos
  estimador.py         rumbo = giroscopio (rápido) + cámara (exacta)
  mundo.py             lee el JSON del contrato v3
  geometria.py         cuentas (distancias, ángulos, "¿el cubo está en la zona?")
  hw.py, red.py, enlace.py   motores/IMU/luz, WiFi + cámara, ESP-NOW
  ideaboard.py         librería OFICIAL de la placa, sin cambios
simulador/             → probar la lógica sin robots
  correr.py            corre muchas partidas al azar y da estadísticas
  en_vivo.py           simula la cancha en tiempo real (se ve en el panel)
herramientas/
  panel.py             PANEL en el navegador: cancha en vivo + pruebas + calibración
  subir_robot.py       copia el programa al robot por USB
```

**Los mismos archivos de lógica corren en el robot y en el simulador.** Si algo anda en el
simulador, el problema que quede en la cancha es de calibración, no de lógica.

## 3. Cómo funciona la estrategia

1. **Reparto:** al arrancar se prueban todas las formas de repartir los cubos entre los dos
   robots y se elige la que termina antes, obligando a que cada robot lleve al menos uno y
   evitando que los caminos de los dos se crucen. Los dos robots hacen la misma cuenta con
   los mismos datos (por eso coinciden aunque falle la radio); si la radio anda, el 10 manda
   el plan y el 11 lo usa.
2. **Ir detrás del cubo:** el robot se para a unas 9,5 celdas del cubo, del lado opuesto a
   su zona, para después llevarlo derecho. La ruta esquiva los otros cubos y al compañero.
3. **Capturar:** avanza despacio hasta que el cubo queda **entre las dos paletas**.
4. **Llevar:** con el cubo enjaulado va hasta un punto frente a la zona y entra recto,
   mirando al borde. Suelta el cubo **centrado en el fondo** de la zona (no empujado hasta el
   borde: ahí sobresale y no cuenta).
5. **Soltar:** retrocede en línea recta (sin girar, para no arrastrar el cubo) y verifica.
6. **Caminos reservados:** cada robot anuncia por radio el camino que le falta. El otro no
   empieza una tarea que lo cruce: espera o se corre (`ESPERAR` / `APARTAR`). Además hay un
   freno de emergencia si los dos se van a tocar.
7. **Si algo sale mal** (se le escapa el cubo, queda trabado, el otro lo tapa) vuelve a
   intentar. Nunca empuja un cubo ya entregado ni uno pegado al borde.

**Simulador** (cientos de partidas al azar): 3 cubos válidos en ~100 % de las partidas, mediana
≈ 30 s, cero choques. También anda sin radio, sin giroscopio, con giroscopio invertido, con
robots lentos o rápidos y con la cámara atrasada.

---

## 4. Probar SIN robots (empezá por acá)

Necesitás Python 3.9+ en la compu (`python --version` en PowerShell).

**Terminal 1** — dentro de la carpeta del repo:
```
python simulador/en_vivo.py
```
**Terminal 2:**
```
python herramientas/panel.py
```
Se abre el navegador. En la terminal 1 escribí `ready` y Enter: a los 5 segundos los
robots virtuales arrancan solos. `nuevo` + `ready` = otra partida con cubos en otro lugar.

Estadísticas de muchas partidas (no hace falta el panel):
```
python simulador/correr.py -n 50
python simulador/correr.py --ver 7      # una partida con detalle y una imagen del recorrido
```

## 4b. Pruebas del robot SIN cámara

Con el robot conectado por USB:
```
python herramientas/subir_robot.py 10 --prueba
```
Abrí Thonny → consola (abajo) → si no muestra nada, hacé clic en la consola y apretá
**Ctrl+D** (reinicia el robot). Seguí las instrucciones: prueba la luz, el giroscopio (girás
el robot con la mano), cada rueda (con el robot levantado), la potencia mínima, un giro de
90° y un tramo recto (en el piso), el WiFi y la radio entre los dos robots. Al final imprime
los valores para `robot_10.py`. Para volver al programa de competencia:
`python herramientas/subir_robot.py 10`.

## 4c. Probar fuera de casa (universidad)

Las redes de la universidad suelen pedir usuario y contraseña o no dejar que dos aparatos
se hablen entre sí: **el robot no puede usarlas**. Usá la laptop como punto de acceso:
Configuración → Red e Internet → **Zona con cobertura inalámbrica móvil** → Editar: nombre,
contraseña y **banda 2,4 GHz** → Activar. En `secretos.py`: ese nombre y contraseña, y
`VISION_HOST = "192.168.137.1"` (la IP que Windows le da a la laptop en su propia zona).

## 5. Preparar los robots (una sola vez)

### 5.1 Librerías en la placa
En el robot, carpeta `lib/` (se ve en Thonny, panel de archivos), tienen que estar: `adafruit_motor`, `neopixel`,
`simpleio`, `adafruit_pixelbuf`, `adafruit_lsm6ds`, `adafruit_register`,
`adafruit_bus_device`. Si falta alguna, `subir_robot.py` avisa. Se bajan del
**CircuitPython Library Bundle** (circuitpython.org/libraries) **de la misma versión mayor**
que dice el archivo `boot_out.txt` del robot.

### 5.2 WiFi
Editá `firmware/rover/secretos.py` (ya está creado con tus datos de antes):
* **El ESP32 solo usa WiFi de 2,4 GHz.** Una red con "5G" en el nombre probablemente no
  sirva. Si el router tiene las dos bandas, usá la de 2,4.
* `VISION_HOST` = IP de la compu donde corre la visión (`ipconfig` → Dirección IPv4).
* `DEPURACION_HOST` = IP de la compu con el panel (normalmente la misma), para ver qué
  piensa cada robot.
* `MODO_PRUEBAS = True` solo en casa.

### 5.3 Subir el programa
La IdeaBoard (ESP32) **no aparece como unidad** en el Explorador de archivos: los archivos
se pasan por el cable, como hace Thonny con "Subir a /". El script lo hace todo de una vez.
Una sola vez: `python -m pip install pyserial`. Después, **con Thonny cerrado** (ocupa el
puerto), conectá **un** robot por USB y, desde la carpeta del repo:
```
python herramientas/subir_robot.py 10 --limpiar
```
(`11` para el otro robot; tiene que coincidir con el marcador ArUco que lleva pegado).
`--limpiar` borra del robot los archivos de la versión anterior.

Abrí la consola serie (Thonny, o la extensión Serial Monitor de VS Code a 115200) para ver
lo que dice. Luces del robot:

| Luz | Significa |
|---|---|
| blanca → **roja 2 s** | arrancando / midiendo el giroscopio: **no moverlo** |
| naranja titilando | conectando al WiFi |
| violeta | no le llega la telemetría (revisar VISION_HOST / firewall) |
| azul / celeste | IDLE / READY: esperando |
| verde / celeste fuerte | trabajando / llevando un cubo |
| naranja fijo | esperando que pase el compañero |
| verde fijo al final | no quedan cubos |

> Si dice que hay varios puertos: `python herramientas/subir_robot.py --puertos` y después
> agregá `--puerto COM5` (el que corresponda). Si en la consola aparece `MemoryError`,
> avisale a Claude (se puede compilar a .mpy para usar menos memoria).

## 6. Probar CON los robots y la cámara

### 6.1 La cancha en casa
* Tablero de 1×1 m, los 4 marcadores de 10 cm (IDs 0–3) en las esquinas, **impresos al
  100 %**, y los de 40 mm (IDs 10 y 11) arriba de cada robot. Ver
  `vision-system/MONTAJE.md` del repo oficial (el marcador 0 arriba a la izquierda mirando
  desde la cámara, después 1, 2, 3 en sentido horario).
* La webcam del kit **mirando hacia abajo**, lo más alto posible (la oficial está a 2,1 m) y
  que se vean los 4 marcadores.

### 6.2 Correr el sistema de visión OFICIAL (en Windows)
Necesita Python 3.10 o más nuevo. En PowerShell, dentro de `Vision-Rover-Challenge\vision-system`:
```
py -3 -m venv .venv
.venv\Scripts\python -m pip install -r vision\requirements.txt
.venv\Scripts\python -m vision.sistema --ventana
```
La primera vez hay que **calibrar la cámara** (`vision-system/PUESTA_A_PUNTO.md`, paso a
paso). La ventana muestra lo que ve; con la tecla `r` (o escribiendo `ready`) arranca una
ronda. Si Windows pregunta por el firewall, marcá redes **privadas y públicas** (la red
del celular suele quedar como "pública").

Ese mismo programa publica en el puerto 2026: los robots y el panel se conectan solos.

**Cada vez que lo abras:**
* **Calibrá los colores.** Cuando pregunte `¿Calibrar los colores ahora? [s/N]`, poné
  los **tres cubos** dentro de la cancha, separados y sin robots encima, y contestá `s`.
  Si un color no se calibra (en casa el azul no se veía nunca y el verde casi nunca),
  el robot **no puede ir a buscar ese cubo**: para él no existe. Si lo rechaza, probá
  con otra exposición (`-5` da más luz, `-7` menos) y repetí.
* **Mirá qué tan rápida va la cámara.** En el panel, "Telemetría → cámara". En una compu
  común puede dar ~2,5 fotos nuevas por segundo con ~0,4 s de atraso (lo normal del
  sistema oficial son ~20). El robot lo compensa (estima dónde está entre foto y foto con
  el giroscopio y los motores), pero conviene: compu **enchufada al cargador** (en
  batería Windows baja el procesador), en modo de energía "máximo rendimiento" y sin
  otros programas pesados abiertos.

### 6.3 Pruebas en orden (no todo junto)
Con `MODO_PRUEBAS = True` y el panel abierto (`python herramientas/panel.py`):

1. **¿Llega la telemetría?** El panel dice "visión conectada" y muestra los dos robots
   donde están de verdad. Girá un robot a mano: la flecha del panel tiene que girar igual.
1b. **¿El marcador apunta a las paletas?** La visión publica hacia dónde mira el BORDE
   DE ARRIBA del marcador ArUco. Si el marcador se pegó girado (con la parte de arriba
   hacia un costado o hacia atrás), el robot cree que mira para otro lado: da vueltas
   alrededor del cubo, se va hacia el borde o lleva el cubo a otra zona. Se mide con el
   robot en el medio de la cancha mirando hacia adentro y el botón
   **"🧭 Calibrar marcador"**. En "Robots" aparece `marcador: girado N°`. Ese número va en
   `DESFASES_MARCADOR` de `config.py` (los de los DOS robots: cada uno usa también el del
   compañero para no chocarlo). Medido el 3-oct-2026: **los dos dan 0°**. Si se despega
   y se vuelve a pegar un marcador, hay que medirlo otra vez. En el panel, el dibujo del robot ya sale corregido y la
   flecha punteada es hacia dónde cree el robot que miran sus paletas.
2. **¿El robot informa?** En "Robots" aparece su estado. Si no, revisá `DEPURACION_HOST`.
3. **Motores:** "Ir recto y medir". Si va para atrás, cambiá `MOTOR_IZQ_SIGNO` /
   `MOTOR_DER_SIGNO` en `robot_10.py`. Si se curva mucho, ajustá `GANANCIA_DER`. Anotá las
   celdas/s que muestra y ponelas en `CELDAS_POR_S` (`config.py`).
4. **Giros:** "Girar a 90°", 180°... tiene que terminar mirando bien (±4°). Si dice
   "giroscopio INVERTIDO", poné `GIRO_SIGNO = -1.0`.
   Si los giros se quedan cortos o "tiemblan", subí `MOTOR_MINIMO` de 0,02 en 0,02.
5. **AGARRE:** poné un cubo metido entre las paletas tocando el frente y apretá
   "Medir AGARRE". Ese número va en `AGARRE` (`config.py`, hoy 5.0).
6. **Clic en la cancha:** el robot va a ese punto.
7. **Agarrar / Llevar** un color: hace la tarea completa con un cubo.
8. **Estrategia completa:** los dos robots, o mejor todavía: arrancá una ronda en la visión
   (`r`) y que arranquen solos con `RUNNING`, como en la competencia.

Cada vez que cambies algo: `python herramientas/subir_robot.py 10` (y 11).

**Registro de cada prueba.** El panel guarda todo lo que pasa en
`registros/sesion_FECHA_HORA.ndjson` (la cámara, lo que informa cada robot y las órdenes).
Para ver el resumen de la última sesión:
```
python herramientas/analizar_registro.py
```
Dice qué hizo cada robot, si corrigió el marcador, lo más cerca que estuvo del borde,
cuántas vueltas por segundo da su programa y por qué frenó. Es lo primero que hay que
mirar (o mandarle a quien te ayude) cuando algo sale raro.

---

## 7. Checklist del día de la competencia

- [ ] `secretos.py` con el **WiFi de la competencia** (2,4 GHz) y la **IP de la PC de visión**
      que dé la organización. **Preguntarles ambas cosas apenas lleguen.**
- [ ] `MODO_PRUEBAS` en **False** (comentado en secretos.py) y `DEPURACION_HOST = None`.
- [ ] Subido a los dos robots (`subir_robot.py 10` y `11`), con el marcador correcto en cada uno.
- [ ] `DESFASES_MARCADOR` (config.py) con lo que midió "Calibrar marcador" para cada robot.
- [ ] Robots en la salida **mirando hacia adentro de la cancha**.
- [ ] Colores calibrados al abrir la visión (los tres cubos a la vista) y, en el panel,
      cuántas fotos por segundo da la cámara de la organización.
- [ ] Baterías cargadas.
- [ ] Durante `IDLE`: encender los robots **quietos** en la salida (2 s de luz roja), esperar
      luz **azul** (conectados). Luz violeta = no ve la telemetría.
- [ ] Después de `READY`/`RUNNING`: **no tocar nada**. Arrancan solos.
- [ ] Entre intento e intento no hace falta reprogramar: cuando la fase vuelve a IDLE/READY
      se reinician solos.

## 8. Preguntas para la organización

1. ¿El arranque válido es en `READY` o en `RUNNING`? (nosotros arrancamos en RUNNING, que
   es cuando arranca el cronómetro del sistema oficial).
2. Nombre y clave del WiFi de la cancha (¿es de 2,4 GHz?) e IP de la PC de visión.
3. ¿Se puede usar ESP-NOW por difusión? (es lo que recomiendan en `robot.md`).

## 9. Si algo no anda

| Síntoma | Qué mirar |
|---|---|
| Luz violeta, consola "No conecta a la vision" | VISION_HOST, misma red WiFi, firewall de la PC de visión |
| El robot avanza "a saltitos": anda un poco, se para (luz naranja) y sigue | **Es a propósito** ("pare y siga"). Con la cámara de casa (~1,6 fotos/s y 0,65 s de atraso) el robot no se ve mientras anda; si anda mucho a ciegas se va al borde o empuja el cubo de más. Anda un tramo corto, se para hasta verse en una foto nítida y sigue. Con una cámara rápida (como la oficial) no se para |
| Cambiar de red WiFi (router o celular) | `WIFI_REDES` en secretos.py: el robot prueba las redes en orden. La compu tiene que estar en la MISMA red. Con el panel abierto, el robot encuentra solo la IP de la compu |
| Luz violeta a mitad de una prueba | se cortó el WiFi con la visión. El robot reintenta solo y, si en 12 s no vuelve, **se reinicia solo** (luz blanca → roja → azul; no tocarlo mientras está en rojo). En el panel, la línea "WiFi" dice la señal (peor que -75 dBm = débil: acercar el router) y el último error. `analizar_registro.py` cuenta los cortes |
| El robot se reinició solo en medio de una prueba | el "perro guardián": el programa se trabó más de 8 s (casi siempre, la radio WiFi). Es a propósito: antes quedaba girando sin control. El panel lo muestra como "se reinició solo (WATCHDOG)" |
| La cámara ve poco el cubo azul (el analizador dice "blue 8%") | luz baja: el azul se ve casi negro. Probar `--exposicion -3` (más clara) si las esquinas se siguen viendo, recalibrar los colores con los 3 cubos en el centro, o probar de día |
| Consola "AVISO: no encontre la IMU" | anda igual, solo con la cámara (gira más lento) |
| "Llevar azul/verde" no hace nada, o no agarra ese color | la cámara no ve ese cubo: calibrar los colores al abrir la visión (6.2) |
| Zigzaguea, se pasa del cubo, lo suelta lejos de la zona | cámara lenta: mirar "fotos nuevas/s" en el panel y el "atraso medido" del robot (6.2) |
| Da vueltas alrededor del cubo, se va hacia el borde o lleva el cubo a otra zona | marcador pegado girado: botón "Calibrar marcador" y `DESFASES_MARCADOR` (6.3, paso 1b) |
| Gira para el lado equivocado o da vueltas sin parar | `GIRO_SIGNO`, o los cables de un motor invertidos (`MOTOR_*_SIGNO`) |
| Se curva al ir recto | `GANANCIA_DER` |
| Se queda "temblando" sin llegar | subir `MOTOR_MINIMO` |
| El cubo se le escapa al llevarlo | bajar `VEL_LLEVANDO` / `GIRO_MAX_LLEVANDO`, revisar `AGARRE` |
| Los dos quedan esperando | mirar en el panel el campo "frena: ..." de cada uno |
| Consola "[info] 8 vueltas/s" (muy lento) | el código ya baja la velocidad solo; si es muy poco, compilar a .mpy |
