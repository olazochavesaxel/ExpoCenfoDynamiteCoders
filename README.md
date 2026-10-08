# ExpoCenfoDynamiteCoders — Vision Rover Challenge

Código del equipo **DynamiteCoders** para el
[Vision Rover Challenge](https://github.com/Universidad-Cenfotec/Vision-Rover-Challenge)
(Expo Cenfo 2026, Universidad Cenfotec): dos CenfoBots que, con la telemetría de la
cámara oficial (contrato **v3**), reparten, buscan y llevan los cubos a sus zonas de forma
autónoma y coordinada.

**Empezá por [`docs/GUIA.md`](docs/GUIA.md).**

| Carpeta | Qué es |
|---|---|
| `firmware/rover/` | Programa de competencia (CircuitPython). Va adentro de cada robot. |
| `simulador/` | Cancha simulada que corre **el mismo código** de los robots. |
| `herramientas/panel.py` | Panel en el navegador: cancha en vivo, estado de cada robot, pruebas y calibración. |
| `herramientas/subir_robot.py` | Copia el programa a un robot por USB. |
| `firmware/pruebas/` | Pruebas sueltas de hardware (motores, IMU, color). |
| `firmware/code.py` | Diagnóstico general del robot (luz, IR, ultrasonido, motores). |
| `archivo/` | Versión anterior del código (solo referencia). |

Rápido:
```
python simulador/en_vivo.py          # terminal 1, después escribir: ready
python herramientas/panel.py         # terminal 2 (abre el navegador)
python simulador/correr.py -n 50     # estadísticas de 50 partidas simuladas
python herramientas/subir_robot.py 10   # con el robot 10 conectado por USB
```

## Flujo de trabajo del equipo
- Nadie hace push directo a `main`: cada quien en su rama (`git checkout -b tarea`) y Pull Request.
- `firmware/rover/secretos.py` (WiFi) **no se sube** a GitHub; cada uno lo crea desde
  `secretos_ejemplo.py`.
