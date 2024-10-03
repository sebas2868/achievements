import numpy as np
import matplotlib.pyplot as plt

def dh_transform(theta, d, a, alpha):

    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
def forward_kinematics(theta1, d1, theta2, theta3):
    # Parámetros D-H d
    d = [0.045, d1, -0.0575, 0.0811]
    a = [0, 0.228, 0.1365, 0]
    alpha = [0, 0, np.pi, 0]
    theta = [theta1, 0, theta2, theta3]
    T1 = dh_transform(theta[0], d[0], a[0], alpha[0])
    T2 = dh_transform(theta[1], d[1], a[1], alpha[1])
    T3 = dh_transform(theta[2], d[2], a[2], alpha[2])
    T4 = dh_transform(theta[3], d[3], a[3], alpha[3])
    T_final = T1 @ T2 @ T3 @ T4
    #print("Matriz actual")
    #print(T1@T2)

    #print("Matriz anterior")
    #print(T1@T2@T3)
    #print("resultante")
    #print(T1@T2@T3@T4)
    
    # Coordenadas del efector. final
    efector_final = T_final[0:3,3]
   
    return efector_final
def fkine(q):
    x = -0.1365*np.sin(q[0])*np.sin(q[2]) + 0.1365*np.cos(q[0])*np.cos(q[2]) + 0.228*np.cos(q[0])
    y = 0.1365*np.sin(q[0])*np.cos(q[2]) + 0.228*np.sin(q[0]) + 0.1365*np.sin(q[2])*np.cos(q[0]) 
    z = q[1] - 0.0936
    return np.array([x,y,z])

# Definición de longitudes de los eslabones
L1 = 0.228
L2 = 0.1365
#===================================CINEMÁTICA MTH====================================
def hallar_angulos_MTH(X, Y):
    D = X**2 + Y**2
    C2 = (D - L1**2 - L2**2) / (2 * L1 * L2)
    q2 = np.arctan2(np.sqrt(1 - C2**2), C2)
    C1 = L2*np.sin(q2)*(Y/X)+L2*np.cos(q2)+L1
    C1 = C1 / (X + (Y**2/X))
    q1 = np.arctan2(np.sqrt(1 - C1**2), C1)
    print("q1MTH:",q1)
    print("q2MTH:",q2)
    #q1 = np.arctan2(Y, X) - np.arctan2(L2 * np.sin(q2), L1 + L2 * np.cos(q2))
    #q1 = np.arctan2(-np.sqrt(1 - C1**2), C1)
    return q1, q2
#================================CINEMÁTICA ALGEBRAÍCO================================
def hallar_angulos_ALG(X, Y):
    q1 = np.arctan2(Y,X) + np.arccos(np.clip((53.67*(X**2 + Y**2)+1.79)/(24.46*np.sqrt(X**2 + Y**2)),a_min=-1,a_max=1))
    q2 = np.arctan2(Y*np.cos(q1)-X*np.sin(q1),X*np.cos(q1)+Y*np.sin(q1)-0.228)
    print("q1ALG:",q1)
    print("q2ALG:",q2)
    return q1, q2
#============================CINEMÁTICA NUMÉRICA GRADIENTE============================
def hallar_angulos_NUMGRAD(X, Y):
    delta = 0.001
    error= []
    xd = np.array([X, 0.0, Y])
    # Valor inicial en el espacio articular
    q = np.array([0.0, 0.0936, 0.0])
    alpha = 1.3
    # Parámetros de iteración
    epsilon = 1e-3
    max_iter = 2000
    angulo_final = 1.5
    i = 0
    # Iteraciones: Método de Newton
    while True:
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        # Matriz Jacobiana
        JT = 1/delta*np.array([
            fkine([q1+delta, q2, q3]) - fkine([q1,q2,q3]),
            fkine([q1, q2+delta, q3]) - fkine([q1,q2,q3]),
            fkine([q1, q2, q3+delta]) - fkine([q1,q2,q3])
            ])
        J = JT.transpose()
        # Posición del extremo del robot
        Nx = np.cos(angulo_final)
        Ny = -np.sin(angulo_final)
        q4 = np.arctan2(Nx,Ny) - q1 - q3
        f=forward_kinematics(q1,q2,q3,q4)
        # Error entre la posición deseada y la actual
        e = xd - f
        error.append(abs(e))
        # Actualizar las posiciones articulares
        q = q + alpha*np.dot(J.transpose(), e)
        # Condición de término
        if np.linalg.norm(e) < epsilon:
            break
        i+=1
        if i == max_iter:
            print("Límite de iteraciones alcanzado")
            break
        # Graficar el error acumulado
    print("q1NUMG:",q[0])
    print("q2NUMG:",q[2])
    plt.plot(range(len(error)), error)
    plt.xlabel('Iteraciones')
    plt.ylabel('Norma del Error')
    plt.title('Convergencia del Error en el Método de Gradiente')
    plt.grid(True)
    plt.show()
        
    return q[0],q[2]

# Función para calcular las posiciones de los eslabones
def obtener_posiciones(q1, q2):
    x0, y0 = 0, 0  # origen
    x1, y1 = L1 * np.cos(q1), L1 * np.sin(q1)  # primer eslabón
    x2, y2 = x1 + L2 * np.cos(q1 + q2), y1 + L2 * np.sin(q1 + q2)  # segundo eslabón
    return [(x0, y0), (x1, y1), (x2, y2)]

# Punto de prueba
X_prueba, Y_prueba = 0.1, 0.3

# Obtener los ángulos de cada método
q1_MTH, q2_MTH = hallar_angulos_MTH(X_prueba, Y_prueba)
q1_ALG, q2_ALG = hallar_angulos_ALG(X_prueba, Y_prueba)
q1_NUMGRAD, q2_NUMGRAD = hallar_angulos_NUMGRAD(X_prueba, Y_prueba)

# Obtener las posiciones de los eslabones
posiciones_MTH = obtener_posiciones(q1_MTH, q2_MTH)
posiciones_ALG = obtener_posiciones(q1_ALG, q2_ALG)
posiciones_NUMGRAD = obtener_posiciones(q1_NUMGRAD, q2_NUMGRAD)

# Función para graficar posiciones
def graficar_posiciones(posiciones, titulo, color):
    x_vals, y_vals = zip(*posiciones)
    plt.plot(x_vals, y_vals, '-o', label=titulo, color=color)
    plt.scatter(x_vals, y_vals, color=color)  # Puntos de articulación
    plt.xlim([-0.5, 0.5])
    plt.ylim([-0.5, 0.5])
    plt.gca().set_aspect('equal', adjustable='box')

# Crear el gráfico
plt.figure()

# Graficar cada uno de los resultados
graficar_posiciones(posiciones_MTH, 'Método MTH', 'blue')
graficar_posiciones(posiciones_ALG, 'Método Algebráico', 'green')
graficar_posiciones(posiciones_NUMGRAD, 'Método Numérico por Gradiente', 'red')

# Configuración del gráfico
plt.title("Posiciones de los Eslabones para Cada Método")
plt.xlabel("Posición X")
plt.ylabel("Posición Y")
plt.legend()
plt.grid(True)

# Mostrar el gráfico
plt.show()
