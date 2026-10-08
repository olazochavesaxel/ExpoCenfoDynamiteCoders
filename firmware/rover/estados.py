# estados.py — nombres de estados compartidos por las partes del cerebro.

LLEVANDO = ("LLEVAR", "SOLTAR")
CON_CUBO = ("CAPTURAR", "LLEVAR", "SOLTAR", "VERIFICAR")
ACTIVOS = ("IR", "ALINEAR", "CAPTURAR", "LLEVAR", "SOLTAR")
RANGO = {"CAPTURAR": 3, "LLEVAR": 3, "SOLTAR": 3, "ALINEAR": 2, "IR": 1}
PRIORIDAD = {"LLEVAR": 3, "SOLTAR": 3, "APARTAR": 0, "CAPTURAR": 2, "ALINEAR": 2,
             "IR": 1, "VERIFICAR": 1, "DESTRABAR": 1}
