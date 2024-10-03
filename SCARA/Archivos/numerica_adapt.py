import numpy as np
import matplotlib.pyplot as plt

alpha = 1.3
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
    efector_final = T_final[0:3,3]
    return efector_final

def fkine(q):
    x = -0.1365*np.sin(q[0])*np.sin(q[2]) + 0.1365*np.cos(q[0])*np.cos(q[2]) + 0.228*np.cos(q[0])
    y = 0.1365*np.sin(q[0])*np.cos(q[2]) + 0.228*np.sin(q[0]) + 0.1365*np.sin(q[2])*np.cos(q[0]) 
    z = q[1] - 0.0936
    return np.array([x,y,z])

def newton(J,e):
    return np.dot(np.linalg.inv(J), e)
def gradient(J,e):
    return alpha*np.dot(J.transpose(), e)

# Definición de longitudes de los eslabones
L1 = 0.228
L2 = 0.1365

def hallar_angulos_NUM(X, Y,m):
    if m == 1:
        print("Newton")
        metodo = newton
    else:
        print("Gradiente")
        metodo = gradient
    delta = 0.001
    error= []
    xd = np.array([X, Y, 0.0])
    # Valor inicial en el espacio articular
    q = np.array([0.001, 0.0936, 0.001])
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
        q = q + metodo(J,e)
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


