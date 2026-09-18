# ExpoCenfoDynamiteCoders — Vision Rover Challenge

Repositorio del equipo para el **Vision Rover Challenge** de Expo Cenfo 2026
(Universidad Cenfotec). Reto oficial y documentación completa:
https://github.com/Universidad-Cenfotec/Vision-Rover-Challenge

## Qué hay en este repo

```
firmware/
├── ideaboard.py          # Librería base del hardware (copiada del repo oficial, no modificar)
├── code.py                # Diagnóstico general: NeoPixel, IR, ultrasónico y motores
└── pruebas/
    ├── prueba_color.py    # Prueba opcional del sensor de color (ver advertencia en el archivo)
    └── prueba_imu.py       # Prueba opcional del acelerómetro/giroscopio
```

Todavía no hay lógica de competencia (visión, navegación, coordinación entre
los dos rovers) — eso se agrega a medida que avancemos.

## Cómo se prueba un CenfoBot

CircuitPython corre el archivo que se llame exactamente `code.py` en la raíz
de la unidad `CIRCUITPY` que aparece cuando conectás el robot por USB. La
carpeta de este repo (donde trabajamos en Git) **no es** esa unidad — hay que
copiar los archivos de un lado al otro.

1. Conectá el robot por USB. Debería aparecer una unidad llamada `CIRCUITPY`.
2. Copiá **ambos** `firmware/ideaboard.py` y `firmware/code.py` a la raíz de
   esa unidad (no dentro de subcarpetas).
3. Apenas termine de copiarse `code.py`, el robot reinicia solo y empieza el
   diagnóstico. Para ver lo que va imprimiendo, abrí la consola serial
   (en Thonny aparece sola; en VS Code, con la extensión **Serial Monitor**).

`code.py` corre, en orden: una luz de prueba, lectura de los 4 sensores
infrarrojos, lectura del sensor ultrasónico y una secuencia corta de
motores. **Antes de correrlo, apoyá el robot sobre algo para que las ruedas
no toquen el piso** — la parte de motores sí mueve el robot.

Las pruebas de `firmware/pruebas/` son opcionales y se prueban una por vez:
copiá esa (además de `ideaboard.py`) a `CIRCUITPY` **renombrándola** a
`code.py`.

### Si algo no corre / da error de librería faltante

CircuitPython necesita, además de estos archivos, ciertas librerías dentro
de una carpeta `lib/` en la unidad `CIRCUITPY` (`adafruit_motor`, `neopixel`,
`simpleio`, `hcsr04`, y para el IMU `adafruit_lsm6ds`). Si el kit vino
configurado de fábrica por la organización ya deberían estar. Si un `import`
falla, es momento de revisar esa carpeta antes que el código.

## Flujo de trabajo del equipo

- No se hace push directo a `main`. Cada quien trabaja en su propia rama:
  ```
  git checkout -b nombre-de-la-tarea
  ```
- Al terminar, se abre un Pull Request en GitHub para juntar los cambios.
- Evitemos que dos personas editen `code.py` al mismo tiempo en ramas
  distintas — es el archivo que más fácil genera conflictos.

## Reglamento y contrato de telemetría

Antes de programar lógica de competencia, repasar:
- `reglamento.md` y `el_reto.md` del repo oficial (condiciones de autonomía).
- `vision-system/contrato/CONTRATO.md` del repo oficial (formato exacto de
  la telemetría TCP/NDJSON que va a consumir cada rover).
