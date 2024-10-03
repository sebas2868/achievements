import numpy as np
import matplotlib.pyplot as plt

# Definición de longitudes de los eslabones
L1 = 10
L2 = 20

# Función para calcular los ángulos q1 y q2 (cinemática inversa)
def hallar_angulos(X, Y):
    D = X**2 + Y**2
    C2 = (D - L1**2 - L2**2) / (2 * L1 * L2)
    q2 = np.arctan2(np.sqrt(1 - C2**2), C2)
    q1 = np.arctan2(Y, X) - np.arctan2(L2 * np.sin(q2), L1 + L2 * np.cos(q2))
    return q1, q2

# Función para calcular las posiciones de los eslabones
def obtener_posiciones(q1, q2):
    x0, y0 = 0, 0  # origen
    x1, y1 = L1 * np.cos(q1), L1 * np.sin(q1)  # primer eslabón
    x2, y2 = x1 + L2 * np.cos(q1 + q2), y1 + L2 * np.sin(q1 + q2)  # segundo eslabón
    return [(x0, y0), (x1, y1), (x2, y2)]

# Simulación en un rango de X y Y
X_values = np.linspace(-10, 20, 100)
Y_values = np.linspace(10, 20, 100)

# Crear la figura para la animación con grilla
fig, ax = plt.subplots()
ax.set_xlim(-30, 30)
ax.set_ylim(-30, 30)
ax.set_aspect('equal')
ax.grid(True)  # Activamos la grilla

# Inicializamos las líneas y puntos
line, = ax.plot([], [], 'o-', lw=2)
end_point, = ax.plot([], [], 'ro', lw=2)  # Punto final en color rojo

# Función de inicialización
def init():
    line.set_data([], [])
    end_point.set_data([], [])
    return line, end_point

# Función para actualizar la animación
def update(frame):
    X = X_values[frame]
    Y = Y_values[frame]
    q1, q2 = hallar_angulos(X, Y)
    positions = obtener_posiciones(q1, q2)
    line.set_data([p[0] for p in positions], [p[1] for p in positions])
    end_point.set_data(positions[-1][0], positions[-1][1])  # Definimos el punto final
    return line, end_point

# Animación con punto final visible
from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig, update, frames=len(X_values), init_func=init, blit=True, interval=1/3*50, repeat=False)

plt.show()