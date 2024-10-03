import sympy as sp

sp.init_printing(use_unicode=True)

# Definir variables simbólicas
q1, q2, q3, q4, q1_aux, q2_aux, q3_aux, q4_aux = sp.symbols('q1 q2 q3 q4 q1_aux q2_aux q3_aux q4_aux')
Nx, Ny, Nz = sp.symbols('Nx Ny Nz')
Ox, Oy, Oz = sp.symbols('Ox Oy Oz')
Ax, Ay, Az = sp.symbols('Ax Ay Az')
Px, Py, Pz = sp.symbols('Px Py Pz')

valores_matrices = []
valores_matrices_DH=[]
valores_matrices_DH_inv=[]

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
    generar_matriz_DH(theta,n_articulacion,d,a,alpha)

def generar_matriz_DH(theta, n_articulacion, d=0, a=0, alpha=0):
    
    matriz_origen_joint = sp.Matrix([
        [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
        [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
        [0, sp.sin(alpha), sp.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    
    R = matriz_origen_joint[:3, :3]
    p = matriz_origen_joint[:3, 3]
    R_inv = R.T 
    p_inv = -R_inv * p 
    matriz_inversa = sp.eye(4)  
    matriz_inversa[:3, :3] = R_inv  
    matriz_inversa[:3, 3] = p_inv  
    matriz_inversa.simplify()

    valores_matrices_DH_inv.insert(n_articulacion,matriz_inversa)
    valores_matrices_DH.insert(n_articulacion,matriz_origen_joint)

matriz_transf_final = sp.Matrix([
    [Nx, Ox, Ax, Px],
    [Ny, Oy, Ay, Py],
    [Nz, Oz, Az, Pz],
    [0, 0, 0, 1]
])

generar_matriz_articulacion(q1, 0, d=0.045)
generar_matriz_articulacion(0, 1, a=0.228, d=q2)
generar_matriz_articulacion(q3, 2, d=-0.0575, a=0.1365, alpha=sp.pi)
generar_matriz_articulacion(q4, 3, d=0.0811)


# Ecuaciones
print("Primera: ")
aux_m1 = valores_matrices_DH_inv[0] @ matriz_transf_final
aux_m2 = valores_matrices_DH[1] @ valores_matrices_DH[2] @ valores_matrices_DH[3]
aux1 = aux_m1[0,3]
aux2 = aux_m2[0,3]
aux1q1 = aux1
ec1 = sp.Eq(aux1,aux2).simplify()
print("ecuacion 1")
sp.pretty_print(ec1)
aux1 = aux_m1[1,3]
aux2 = aux_m2[1,3]
aux2q1= aux1
ec2 = sp.Eq(aux1,aux2).simplify()
print("ecuacion 2")
sp.pretty_print(ec2)
aux1 = aux_m1[2,3]
aux2 = aux_m2[2,3]
ec3 = sp.Eq(aux1,aux2).simplify()
print("ecuacion 3")
sp.pretty_print(ec3)

# print("Segunda: ")
# aux_m1 = valores_matrices_DH_inv[1] @ valores_matrices_DH_inv[0] @ matriz_transf_final
# aux_m2 = valores_matrices_DH[2] @ valores_matrices_DH[3]
# aux1 = aux_m1[0,3]
# aux2 = aux_m2[0,3]
# ec4 = sp.Eq(aux1,aux2).simplify()
# print("ecuacion 4")
# sp.pretty_print(ec4)
# aux1 = aux_m1[1,3]
# aux2 = aux_m2[1,3]
# ec5 = sp.Eq(aux1,aux2).simplify()
# print("ecuacion 5")
# sp.pretty_print(ec5)
# aux1 = aux_m1[2,3]
# aux2 = aux_m2[2,3]
# ec6 = sp.Eq(aux1,aux2).simplify()
# print("ecuacion 6")
# sp.pretty_print(ec6)

# print("Tercera: ")
# aux_m1 = valores_matrices_DH_inv[2] @ valores_matrices_DH_inv[1] @ valores_matrices_DH_inv[0] @ matriz_transf_final
# aux_m2 = valores_matrices_DH[3]
# aux1 = aux_m1[0,3]
# aux2 = aux_m2[0,3]
# ec7 = sp.Eq(aux1,aux2).simplify()
# print("ecuacion 7")
# sp.pretty_print(ec7)
# aux1 = aux_m1[1,3]
# aux2 = aux_m2[1,3]
# ec8 = sp.Eq(aux1,aux2).simplify()
# print("ecuacion 8")
# sp.pretty_print(ec8)
# aux1 = aux_m1[2,3]
# aux2 = aux_m2[2,3]
# ec9 = sp.Eq(aux1,aux2).simplify()
# print("ecuacion 9")
# sp.pretty_print(ec9)


# Solucion
print("\nPARA Q1")
print("De la ecuacion 1 y 2")
ecq1 = sp.Eq(aux1q1,aux2q1+0.228).simplify()
sp.pretty_print(ecq1)
solution = sp.solve(ecq1,q1)
q1 = solution
print("q1 = \n")
sp.pretty_print(solution)

print("\nPARA Q2")
print("De la ecuacion 3:")
ecq2 = ec3
sp.pretty_print(ecq2)
solution = sp.solve(ecq2,q2)
q2 = solution
print("q2 = ")
sp.pretty_print(solution)

print("\nPARA Q3")
print("De la ecuacion 2")
ecq3 = ec2
sp.pretty_print(ecq3)
solution = sp.solve(ecq3,q3)
q3 = solution
print("q3 = ")
sp.pretty_print(solution)






#aux1 = valores_matrices_DH_inv[1] @ valores_matrices_DH_inv[0]
#aux2 = valores_matrices_DH[2] @ valores_matrices_DH[3] 
#sp.pretty_print(aux1)
#sp.pretty_print(aux2)