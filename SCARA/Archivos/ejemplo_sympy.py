import sympy as sp

# Definir variables simbólicas
x = sp.symbols('x')

# Crear una matriz simbólica
M = sp.Matrix([
    [sp.sin(x), 1],
    [0, sp.cos(x)]
])

# Operaciones simbólicas, como obtener la transpuesta o el determinante
M_transpuesta = M.transpose()
det_M = M.det()

# Imprimir los resultados
print("Matriz:")
sp.pprint(M)
print("\nTranspuesta de M:")
sp.pprint(M_transpuesta)
print("\nDeterminante de M:")
sp.pprint(det_M)


# Definir las variables simbólicas
q1, Nx, Ny, Px = sp.symbols('q1 Nx Ny Px')

# Definir la ecuación
equation = Nx * sp.cos(q1) - Ny * sp.sin(q1) - Px

# Despejar q1
solution = sp.solve(equation, q1)

# Mostrar la solución
sp.pprint(solution)

