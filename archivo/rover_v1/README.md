# Logica de competencia (Vision Rover Challenge)

Primera version completa de la estrategia autonoma, basada en:

- `vision-system/contrato/CONTRATO.md` (v2) del repo oficial, para leer la
  telemetria de la camara.
- `solucion_simple.md` del repo oficial, para el algoritmo greedy.
- Los ejemplos oficiales de `codigos/` (`move_heading.py`, `turn_angle.py`,
  `espnow_bidirectional.py`, `wifi_command_receiver.py`) para los patrones
  de motores, WiFi y ESP-NOW en CircuitPython.

## Archivos

| Archivo | Que hace |
|---|---|
| `config.py` | Constantes a llenar (WiFi, IP de vision, ID del robot, MAC del compañero). |
| `vision_client.py` | Conecta WiFi y el socket TCP a la vision, entrega el ultimo mensaje NDJSON. |
| `geometry.py` | La regla oficial de "cubo en su zona", distancias, rumbos. |
| `coordination.py` | ESP-NOW entre los dos rovers, para no ir por el mismo cubo. |
| `motion.py` | Girar y avanzar usando la posicion/orientacion que da la vision. |
| `strategy.py` | El bucle greedy: elegir cubo, ir, empujar, repetir. |
| `main.py` | Punto de entrada. **Renombrar a `code.py` al subirlo al robot.** |

## Como subir esto a un robot con Thonny

CircuitPython importa modulos desde la raiz de la placa, asi que TODOS estos
archivos van sueltos en la raiz (no dentro de una carpeta "rover"), igual que
`ideaboard.py`:

1. Subi cada uno de `config.py`, `vision_client.py`, `geometry.py`,
   `coordination.py`, `motion.py`, `strategy.py` a la raiz del robot con
   "Subir a /" (igual que veniamos haciendo).
2. Hace una copia de `main.py`, renombrala a `code.py`, y subi esa copia
   tambien a la raiz (reemplazando el `code.py` que hubiera).
3. `ideaboard.py` ya deberia estar subido de antes.

## Que hay que llenar/revisar ANTES de probar con vision real

Editar `config.py` (en tu compu, antes de subirlo) con:

- `ROBOT_ID`: 10 u 11 segun el marcador ArUco de ESE robot.
- `WIFI_SSID` / `WIFI_PASSWORD`: la red del lugar donde se prueba.
- `VISION_HOST`: la IP de la compu que corre el sistema de vision (correr
  `ipconfig` ahi, **nunca** poner `127.0.0.1`).
- `PEER_MAC`: la MAC del OTRO robot. Sale sola impresa ("Mi MAC: ...") al
  arrancar `main.py`/`code.py` en ese otro robot.

## Supuestos que hay que confirmar viendo al robot moverse

Esto no se puede verificar sin la camara del sistema de vision funcionando,
asi que quedan marcados en el codigo con comentarios:

1. **Signo del giro** (`motion.girar_hacia`): asumimos que para que `theta`
   crezca hay que hacer lo mismo que `left()` en `test_motores.py`
   (motor_1 atras, motor_2 adelante). Si el robot gira siempre al reves de
   lo pedido, invertir el signo de `direccion` en `motion.py`.
2. **Eje `row`** (`geometry.heading_hacia`): asumimos que `row` crece hacia
   abajo en la imagen, y por eso se invierte para calcular el rumbo. Si el
   robot apunta sistematicamente al lugar "reflejado" del que debería, este
   es el primer lugar donde revisar.
3. **Canal de ESP-NOW** (`coordination.current_wifi_channel`): asumimos que,
   una vez conectado el WiFi, ESP-NOW funciona en ese mismo canal sin tener
   que forzarlo. Si los dos rovers no se reciben entre si estando ambos
   conectados al WiFi de la vision, este es el primer sospechoso.

## Como probar por partes (recomendado, no probar todo junto de una)

1. **Sin vision, sin ESP-NOW**: en Thonny, importar `motion` a mano desde la
   consola y probar `motion.detener()` / mover motores directo, para
   confirmar que el robot sigue respondiendo con estos archivos nuevos.
2. **Con vision, sin estrategia**: correr solo `vision_client` desde la
   consola, conectarse y hacer `print(vc.poll())` varias veces, para
   confirmar que llega telemetria real antes de dejar que el robot se mueva
   solo.
3. **Con vision y movimiento, sin ESP-NOW**: probar `motion.girar_hacia` y
   `motion.avanzar_hacia` a mano con un punto fijo, para calibrar los dos
   supuestos de signos de la seccion anterior.
4. **Los dos rovers con ESP-NOW**: confirmar que cada uno ve el `peer_state`
   del otro (`coordinator.peer_state` despues de `recibir_pendientes()`).
5. Recien ahi, `code.py` completo con `main.py`.

## Lo que falta / se puede mejorar despues

- Evasion de obstaculos y del otro rover usando el sensor ultrasonico como
  respaldo local (ahora mismo la coordinacion es solo "no elegir el mismo
  cubo", sin frenar si se cruzan caminos).
- Calibrar velocidades y tolerancias (`config.py`) con el robot real.
- Manejar el caso de que la vision pierda de vista un cubo por mucho tiempo
  (`age_ms` alto): hoy se sigue usando la ultima posicion conocida tal cual,
  como pide el reglamento, pero sin ningun aviso si esa posicion ya es muy
  vieja.
