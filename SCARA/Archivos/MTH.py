import sympy as sp

# Iniciar la impresión mejorada
sp.init_printing(use_unicode=True)

# Definir variables simbólicas
q1, q2, q3, q4, q1_aux, q2_aux, q3_aux, q4_aux = sp.symbols('q1 q2 q3 q4 q1_aux q2_aux q3_aux q4_aux')
Nx, Ny, Nz = sp.symbols('Nx Ny Nz')
Ox, Oy, Oz = sp.symbols('Ox Oy Oz')
Ax, Ay, Az = sp.symbols('Ax Ay Az')
Px, Py, Pz = sp.symbols('Px Py Pz')

valores_matrices = []

def generar_matriz_articulacion(theta, n_articulacion, d=0, a=0, alpha=0):
    
    matriz_origen_joint = sp.Matrix([
        [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
        [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
        [0, sp.sin(alpha), sp.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    
    if n_articulacion > 0:
        matriz_anterior = valores_matrices[n_articulacion - 1]
    else:
        matriz_anterior = sp.eye(4)  # Matriz identidad
    
    matriz_resultante = matriz_anterior @ matriz_origen_joint
    valores_matrices.append(matriz_resultante)
    
matriz_transf_final = sp.Matrix([
    [Nx, Ox, Ax, Px],
    [Ny, Oy, Ay, Py],
    [Nz, Oz, Az, Pz],
    [0, 0, 0, 1]
])



print("Matriz de transformación final:")
sp.pretty_print(matriz_transf_final)

generar_matriz_articulacion(q1, 0, d=0.045)
generar_matriz_articulacion(0, 1, a=0.228, d=q2)
generar_matriz_articulacion(q3, 2, d=-0.0575, a=0.1365, alpha=sp.pi)
generar_matriz_articulacion(q4, 3, d=0.0811)

aux1 = sp.Matrix.inv(valores_matrices[0]) @ matriz_transf_final
aux2 = valores_matrices[1] @ valores_matrices[2] @ valores_matrices[3]
ecuacion_aux = sp.Eq(aux1,aux2)



print("aux1: ")
sp.pretty_print(aux1)

print("aux2: ")
sp.pretty_print(aux2)


print("solucion: ")
#solution = sp.solve(ecuacion_aux)
#sp.pretty_print(solution)
