# depurar.py — imprime paso a paso un escenario: python simulador/depurar.py SEMILLA T0 T1
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import correr
sem = int(sys.argv[1]); t0 = float(sys.argv[2]); t1 = float(sys.argv[3])
p = correr.Partida(sem, silencio=True)
k = p.k
print("cubos:", {c: tuple(round(x, 1) for x in v) for c, v in k.cubos.items()})
while k.t < t1 and k.fase != "FINISHED":
    p.correr(t_max=k.t + 0.25)
    if k.t < t0:
        continue
    linea = "%6.2f" % k.t
    for rid, c in p.cer.items():
        r = k.rovers[rid]
        linea += " | R%d %-9s %-5s (%4.1f,%4.1f,%4.0f) v=%.2f w=%.2f f=%s o=%s" % (
            rid, c.estado, c.tarea or "-", r.x, r.y, r.th, c.cmd[0], c.cmd[1], getattr(c,"freno",""),
            c.objetivo and "(%.0f,%.0f)" % c.objetivo)
    linea += " | " + " ".join("%s(%.1f,%.1f)" % (cc[0], v[0], v[1]) for cc, v in k.cubos.items())
    print(linea)
for l in p.logs:
    t = float(l.split("[")[1].split("s]")[0]) if "[" in l else 0
    if t0 <= t <= t1:
        print(l)
