import numpy as np
import matplotlib.pyplot as plt

# Definición de longitudes de los eslabones
L1 = 0.228
L2 = 0.1365

# Función para calcular los ángulos q1 y q2 (cinemática inversa)
def hallar_angulos(X, Y):
    D = X**2 + Y**2
    C2 = (D - L1**2 - L2**2) / (2 * L1 * L2)
    q2 = np.arctan2(np.sqrt(1 - C2**2), C2)
    C1 = L2*np.sin(q2)*(Y/X)+L2*np.cos(q2)+L1
    C1 = C1 / (X + (Y**2/X))
    q1 = np.arctan2(np.sqrt(1 - C1**2), C1) 
    #q1 = np.arctan2(Y, X) - np.arctan2(L2 * np.sin(q2), L1 + L2 * np.cos(q2))
    #q1 = np.arctan2(-np.sqrt(1 - C1**2), C1)
    return q1, q2

def hallar_angulos_alg(X, Y):
    q1 = np.arctan2(Y,X) + np.arccos(np.clip((53.67*(X**2 + Y**2)+1.79)/(24.46*np.sqrt(X**2 + Y**2)),a_min=-1,a_max=1))
    q2 = np.arctan2(Y*np.cos(q1)-X*np.sin(q1),X*np.cos(q1)+Y*np.sin(q1)-0.228)
    return q1, q2

# Función para calcular las posiciones de los eslabones
def obtener_posiciones(q1, q2):
    x0, y0 = 0, 0  # origen
    x1, y1 = L1 * np.cos(q1), L1 * np.sin(q1)  # primer eslabón
    x2, y2 = x1 + L2 * np.cos(q1 + q2), y1 + L2 * np.sin(q1 + q2)  # segundo eslabón
    return [(x0, y0), (x1, y1), (x2, y2)]

# Simulación en un rango de X y Y
rho = 0.223
theta = np.linspace(0,2*np.pi,100)
X_values = rho*np.cos(theta)#np.linspace(0.1, 0.3, 100)
Y_values = rho*np.sin(theta)#np.linspace(0.1, 0.3, 100)
#print(X_values)
# Crear la figura para la animación con grilla
fig, ax = plt.subplots()
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_aspect('equal')
ax.grid(True)  # Activamos la grilla
ax.axvline(x=0.223, color='r', linestyle='--')

# Agregar línea horizontal en y = 0.3645
ax.axhline(y=0.223, color='b', linestyle='--')


# Inicializamos las líneas y puntos
line, = ax.plot([], [], 'o-', lw=2)
end_point, = ax.plot([], [], 'ro', lw=2)  # Punto final en color rojo

# Función de inicialización
def init():
    line.set_data([], [])
    end_point.set_data([], [])
    return line, end_point

#PUNTO DE PRUEBA
q1MTH,q2MTH = hallar_angulos(0.1,0.2)
print("========MTH=========")
print("========Q1=========")
print(q1MTH*180/np.pi)
print("========Q2=========")
print(q2MTH*180/np.pi)
#PUNTO DE PRUEBA
q1al,q2al = hallar_angulos_alg(0.1,0.2)
print("========ALGEBRAICO=========")
print("========Q1=========")
print(q1al*180/np.pi)
print("========Q2=========")
print(q2al*180/np.pi)
# Función para actualizar la animación
def update(frame):
    X = X_values[frame]
    Y = Y_values[frame]
    q1, q2 = hallar_angulos_alg(X, Y)
    #print(theta[frame]*180/np.pi)
    # print(np.rad2deg(q1))
    # print(np.rad2deg(q2))
    positions = obtener_posiciones(q1, q2)
    line.set_data([p[0] for p in positions], [p[1] for p in positions])
    end_point.set_data(positions[-1][0], positions[-1][1])  # Definimos el punto final
    return line, end_point

# Animación con punto final visible
from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig, update, frames=len(X_values), init_func=init, blit=True, interval=1/3*50, repeat=False)

plt.show()
# plt.figure()
# plt.plot(q1a)
# plt.figure()
# plt.plot(q2a)
# plt.show()