import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
import csv
import matplotlib.patches as patches

def cargar_datos():
    # Cargar el archivo CSV
    ruta_del_archivo = 'ejercicio_nuevo.csv'  # Reemplaza con la ruta de tu archivo CSV
    df = pd.read_csv(ruta_del_archivo)
    return df

def get_column_names(csv_file_path):
    with open(csv_file_path, mode='r', newline='') as file:
        reader = csv.reader(file)
        # Lee la primera fila
        column_names = next(reader)
    return column_names


def entrenar_som(df,nEpocas=100):
    data_set_1 = np.concatenate((np.eye(df.shape[1]-1)*0.2, df.iloc[:,1:]), axis=0).T
    data_set = data_set_1 / np.linalg.norm(data_set_1, axis=1, keepdims=True) #Normalizacion
    map_size = 15
    mapa = np.random.rand(map_size,map_size,data_set.shape[1])-0.5
    mapa = mapa /  np.linalg.norm(mapa, axis=2, keepdims=True)

    eta_i = 0.4
    radio_i = 4
    radio_f = 1
    #r=1

    def find_vecindad(map_size, x, y, r):
        resultado = np.ones((map_size,map_size), dtype=float)
        for i in range(map_size):
            for j in range(map_size):
                resultado[i, j] = 1*(r>=abs(x-i) or abs(x-i)>=(map_size-r))*(r>=abs(y-j) or abs(y-j)>=(map_size-r))
        return resultado
    
    neuronas_ganadoras = []
    column_names = df.columns.tolist()
    nombres = column_names[1:]

    for t in range(nEpocas):
        eta = eta_i*((0.01/eta_i)**(t/nEpocas))
        indices_aleatorios = np.random.permutation(data_set.shape[0])
        random_data_set = data_set[indices_aleatorios, :]
        random_nombres = [nombres[i] for i in indices_aleatorios]
        radio = round(radio_i +(radio_f-radio_i)*(t/nEpocas)) #Radio que varia con las epocas
        for m in range(data_set.shape[0]):

            nombre = random_nombres[m]
            sujeto = random_data_set[m,:]
        
            aux = np.tensordot(sujeto, mapa, axes=([0], [2]))
            pos_winer = np.unravel_index(np.argmax(aux), aux.shape)
            neuronas_ganadoras.append((pos_winer, nombre))
            vecindad = find_vecindad(map_size, pos_winer[0], pos_winer[1],radio)
        
            indi_vecindad = indices = np.where(vecindad == 1)
            lista_indices = list(zip(indices[0], indices[1]))
        
            for i in range(len(lista_indices)):
                vecino = mapa[lista_indices[i]]
                mapa[lista_indices[i]] =  vecino + eta*(sujeto- vecino)
                mapa[lista_indices[i]] = mapa[lista_indices[i]] /  np.linalg.norm(mapa[lista_indices[i]])

    return mapa, data_set, data_set_1, neuronas_ganadoras

def graficar_som(mapa,data_set,data_set_1,ruta_del_archivo = 'ejercicio_nuevo.csv'):
    colores = np.load('colores_16.npy')
    
     # Convertir los últimos 4 elementos a una cadena de bits y luego a decimal
    def bin_to_dec(bin_array):
        bin_str = ''.join(str(int(bit)) for bit in bin_array if not np.isnan(bit))
        return int(bin_str, 2)


    def get_column_names(csv_file_path):
        with open(csv_file_path, mode='r', newline='') as file:
            reader = csv.reader(file)
            # Lee la primera fila
            column_names = next(reader)
        return column_names

    column_names = get_column_names(ruta_del_archivo)
    nombres = column_names[1:]
    map_size = mapa.shape[0]

    # Crear una figura y un eje
    fig, ax = plt.subplots(figsize=(16, 16))  # Ajustar el tamaño de la figura

    # Configurar el rango de los ejes
    ax.set_xlim(0, map_size)
    ax.set_ylim(0, map_size)

    # Crear una cuadrícula de 10x10
    ax.set_xticks(np.arange(0, map_size+1, 1))
    ax.set_yticks(np.arange(0, map_size+1, 1))

    # Habilitar la cuadrícula
    ax.grid(which='both')
    for i in range(map_size): 
        for j in range(map_size): 
            aux = np.tensordot(data_set,  mapa[i,j], axes=([1], [0])) #toca revisar 
            pos_winer = np.unravel_index(np.argmax(aux), aux.shape)
            last_4_elements = data_set_1[pos_winer[0],-4:]
            # Convertir los últimos 4 elementos a decimal
            decimal_value = bin_to_dec(last_4_elements)
            ax.text(i+0.5,j+0.5, nombres[pos_winer[0]], ha='center', va='center')
            color = colores[decimal_value]
            rect = patches.Rectangle((i, j), 1, 1, linewidth=1, edgecolor='none', facecolor=color)
            ax.add_patch(rect)

    
def graficar_som_toroide(mapa, data_set, data_set_1, ruta_del_archivo='ejercicio_nuevo.csv'):
    colores = np.load('colores_16.npy')

    def bin_to_dec(bin_array):
        bin_str = ''.join(str(int(bit)) for bit in bin_array)
        return int(bin_str, 2)

    map_size = mapa.shape[0]

    fig = plt.figure(figsize=(16, 16))
    ax = fig.add_subplot(111, projection='3d')

    # Crear la superficie del toroide
    theta = np.linspace(0, 2 * np.pi, map_size)
    phi = np.linspace(0, 2 * np.pi, map_size)
    theta, phi = np.meshgrid(theta, phi)
    R, r = 10, 2.5  # Ajustar el radio menor para que sea más bajo
    X = (R + r * np.cos(phi)) * np.cos(theta)
    Y = (R + r * np.cos(phi)) * np.sin(theta)
    Z = r * np.sin(phi)

    # Crear una matriz para almacenar los colores
    color_matrix = np.zeros((map_size, map_size, 3))

    for i in range(map_size):
        for j in range(map_size):
            aux = np.tensordot(data_set, mapa[i, j], axes=([1], [0]))
            pos_winer = np.unravel_index(np.argmax(aux), aux.shape)

            last_4_elements = data_set_1[pos_winer[0], -4:]
            decimal_value = bin_to_dec(last_4_elements)
            color = colores[decimal_value]

            # Asignar el color a la matriz de colores
            color_matrix[i, j] = color[:3]

    # Dibujar la superficie del toroide con colores sólidos y líneas de contorno
    ax.plot_surface(X, Y, Z, facecolors=color_matrix, edgecolor='k', linewidth=0.5, antialiased=True)
    ax.set_box_aspect([3,3,1])

    plt.show()

def encontrar_5_mas_cercanos(neurona_objetivo, neuronas_ganadoras):
    distancias = {}
    for neurona, nombre in neuronas_ganadoras:
        distancia = np.linalg.norm(np.array(neurona_objetivo) - np.array(neurona))
        if nombre not in distancias or distancia < distancias[nombre]:
            distancias[nombre] = distancia

    distancias_ordenadas = sorted(distancias.items(), key=lambda item: item[1])
    print(f"Los 5 objetos más cercanos a la neurona en la posición {neurona_objetivo} son:")
    for nombre, distancia in distancias_ordenadas[:5]:
        print(f" - {nombre} a una distancia de {distancia:.4f}")

    return distancias_ordenadas[:5]
def agregar_objeto(mapa, nuevo_objeto, ganadoras, dataset):
    nuevo_objeto = np.array(nuevo_objeto, dtype=float)

    matriz = np.eye(dataset.shape[0]) * 0.2

    # Obtener la última fila
    ultima_fila = matriz[-1, :]


    nuevo_objeto = np.concatenate((ultima_fila, nuevo_objeto), axis=0).T

    nuevo_objeto /= np.linalg.norm(nuevo_objeto)
    aux = np.tensordot(nuevo_objeto, mapa, axes=([0], [2]))  # Calcula el producto tensorial entre el nuevo objeto y el mapa
    pos_winer_nuevo = np.unravel_index(np.argmax(aux), aux.shape)
    return encontrar_5_mas_cercanos(pos_winer_nuevo, ganadoras)



