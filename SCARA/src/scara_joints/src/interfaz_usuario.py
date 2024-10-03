#!/usr/bin/env python3

# Librerias
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import ttk
import threading
import time
import numpy as np
from PIL import Image, ImageTk, ImageSequence
import os
import time

class SliderApp(Node): # Crear nodo de ros

    def __init__(self, root): # Funcion de inicio 
        # --------------------------------------------- Parametros de ROS ---------------------------------------------------------------------
        super().__init__('Interfaz_publisher') # iniciar comunicaciones de ros e interfaz
        self.al_arduino_ = self.create_publisher(Float32MultiArray, 'entrada_arduino', 10) # Publicador para enviar datos al Arduino
        self.subscription = self.create_subscription(Float32MultiArray, 'salida_arduino', self.entrada_callback, 10) # subscripcion de los datos medidos por el arduino
        #self.timer = self.create_timer(1/20, self.actualizar) # rate de publicacion de datos

        # --------------------------------------------- Parametros del robot -------------------------------------------------------------------
        self.num_articulaciones_ = 4
        self.valores_matrices = np.zeros((4,4,self.num_articulaciones_))

        # --------------------------------------------- Parametros de interfaz -----------------------------------------------------------------
        self.root = root # Marco Interfaz
        self.root.title("Interfaz Joint Publisher")
        modules_frame = ttk.Frame(root)
        modules_frame.pack(padx=10, pady=5)
        self.sliders = []
        self.labels = []

        # ------------------------------------------- Definicion de articulaciones -------------------------------------------------------------
        self.create_module(modules_frame, 1, -180, 180, -180, 180,a=0, d= 0.045)
        self.create_module(modules_frame, 2, 0, 250, 0, 250, type = "prismatica",a = 0.228)
        self.create_module(modules_frame, 3, -135, 135, -135, 135, d = -0.0575, a = 0.1365, alpha = 180)
        self.create_module(modules_frame, 4, -180, 180, -180, 180, d = 0.0811)
        self.create_module(modules_frame, 5, 0, 90, 0, 90, type = "efector")

    # -------------------------------------------- Funciones auxiliares -------------------------------------------------------------------------
    def graficar_matriz(self, parent, theta, numero_joint, type="revoluta", d=0, a=0, alpha=0): # Funcion para graficar matrices
        for widget in parent.winfo_children():
            widget.destroy() # Limpiar el frame anterior de todos los modulos existentes (Para actualizar las matrices)
            
        # ------------------------------------------- Parametros especificos de cara articulacion -----------------------------------------------
        if type == "prismatica":
            d /= 1000   # Si es prismatica, se debe hacer conversion de mm a metros pues la interfaz esta en milimetros y las matrices en metros (Igual que MRPT)

        theta *= np.pi/180
        alpha *= np.pi/180  # Los angulos de la interfaz se dan en grados, mientras que la matriz los trabaja en radianes

        # ----------------------------------------------Construcción de la matriz DH --------------------------------------------------------------
        matriz_dh = np.round(
            np.array([
                [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]
            ]), 3)
        
        # ---------------------------------------------- Calculo de la matriz con respecto al origen ----------------------------------------------
        matriz_anterior = np.eye(4)
        matriz_actual = matriz_dh 

        if numero_joint>1:
            matriz_anterior = np.array(self.valores_matrices)[:, :, numero_joint - 2] # Leemos el valor de la matriz anterior guardada en la iteracion pasada

        matriz_dh = matriz_anterior @ matriz_actual # Multiplicamos las matrices
        self.valores_matrices[:, :, numero_joint - 1] = matriz_dh # Guardamos la nueva matriz

        # ----------------------------------------------- Graficamos la nueva matriz ------------------------------------------------------------------
        font_size = 80
        matrix_frame = tk.Frame(parent)
        matrix_frame.pack(expand=True, padx=10, pady=10)  # Centra la matriz dentro de su contenedor

        bracket_left = tk.Label(matrix_frame, text="[", font=("Arial", font_size), padx=10, pady=1)
        bracket_left.pack(side="left", anchor="center") # Crea el corchete izquierdo de la matriz

        matrix_content_frame = tk.Frame(matrix_frame)
        matrix_content_frame.pack(side="left", anchor="center") # Centra el cuerpo de la matriz

        rows, cols = matriz_dh.shape # Se grafica cada elemento de la matriz usando un for y redondeando el valor a 3 decimales
        for i in range(rows):
            row_frame = tk.Frame(matrix_content_frame)
            row_frame.pack()

            for j in range(cols):
                label = tk.Label(row_frame, text=np.round(matriz_dh[i, j], 3), font=("Arial", int(font_size / 8)), borderwidth=0, width=5, height=2)
                label.pack(side="left", padx=10, pady=1) 

        bracket_right = tk.Label(matrix_frame, text="]", font=("Arial", font_size), padx=10, pady=1)
        bracket_right.pack(side="left", anchor="center") # Creo el corchete derecho de la matriz


    def actualizar(self):  # Funcion encargada de publicar los valores que leera el arduino
        joints_val_ = Float32MultiArray()
        joints_val_.data = [slider.get() for slider in self.sliders]
        self.al_arduino_.publish(joints_val_)
        self.get_logger().info(f"Publicando valores de sliders: {joints_val_.data}")
        time.sleep(0.1) 

    def entrada_callback(self, msg): # Funcion encargada de leer los valores que manda el arduino (y de convertirlos a las unidades necesarias)
        time.sleep(0.1)
        data_list = list(msg.data)

        for i in range(min(len(data_list), len(self.labels))):
            self.labels[i].config(text=f"Valor medido en Joint {i + 1}: {data_list[i]:.3f}")
            self.get_logger().info(f"Datos recibidos: Joint {i + 1} = {data_list[i]:.3f}")

    # La siguiente funcion crea el modulo de una joint, los modulos son apilables entre si lo que hace que este codigo este generalizado para cualquier robot una vez obtenida la matriz DH
    def create_module(self, parent_frame, module_number, initial_min, initial_max, min_allowed, max_allowed, initial_value = 0, type = "revoluta", d = 0.0, a = 0.0, alpha = 0.0): 
        # Se le debe dar el parent (marco en el que ira dentro de la interfaz), el numero del modulo, un valor limite inicial para la joint que el usuario puede modificar,
        # un limite minimo permitido por seguridad del robot, los mismos conceptos para los limites maximos, un valor inicial para la joint, el tipo de la joint y los valores d, a y alpha segun correspondan.
        # (Notese que para las revolutas theta viene siendo el valor inicial con el que se inicializa la joint.) Este codigo permite valores por defecto, los cuales se usaran a menos que se especifique otro.
        
        def update_value_from_slider(event): # Actualizar cuadro de texto desde el slider
            value_entry.delete(0, tk.END)
            value_entry.insert(0, str(round(slider.get(), 3)))

        def update_slider_from_entry(*args): # Actualizar slider desde el cuadro de texto
            try:
                new_value = float(value_entry.get())
                if new_value < slider.cget('from'):
                    new_value = slider.cget('from')
                elif new_value > slider.cget('to'):
                    new_value = slider.cget('to')
                new_value = max(min_allowed, min(new_value, max_allowed))
                slider.set(new_value)
                value_entry.delete(0, tk.END)
                value_entry.insert(0, str(round(new_value, 3)))
                self.actualizar()
            except ValueError:
                pass

        def on_enter_pressed(event): # Actualizar slider al presionar enter
            update_slider_from_entry()

        def adjust_slider(): # Si se ingresan valores no permitidos se corrigen a los mas cercanos permitidos de forma automatica, de lo contrario se actualizan con los valores ingresados
            try:
                min_value = float(min_entry.get())
                max_value = float(max_entry.get())
            except ValueError:
                print("Por favor, ingrese valores válidos.")
                return
            
            min_value = max(min_value, min_allowed)
            max_value = min(max_value, max_allowed)
            if min_value > max_value:
                min_value = max_value

            slider.config(from_=min_value, to=max_value)
            current_value = slider.get()
            if current_value < min_value:
                current_value = min_value
            elif current_value > max_value:
                current_value = max_value

            slider.set(current_value)
            min_entry.delete(0, tk.END)
            min_entry.insert(0, str(round(min_value, 3)))
            max_entry.delete(0, tk.END)
            max_entry.insert(0, str(round(max_value, 3)))
            print(f"Límites del Joint {module_number} ajustados a {min_value} - {max_value}")

        def reset_defaults(): # Devuelve los limites a sus valores por defecto
            slider.config(from_=initial_min, to=initial_max)
            min_entry.delete(0, tk.END)
            min_entry.insert(0, str(initial_min))
            max_entry.delete(0, tk.END)
            max_entry.insert(0, str(initial_max))
            current_value = initial_value
            if current_value < initial_min:
                current_value = initial_min
            elif current_value > initial_max:
                current_value = initial_max
            slider.set(current_value)
            value_entry.delete(0, tk.END)
            value_entry.insert(0, str(round(current_value, 3)))

        def actualizar_matriz(): # Funcion que actualiza la matriz mediante un boton
            data = [slider.get() for slider in self.sliders]
            if type == "revoluta":
                self.graficar_matriz(right_frame,float(data[module_number]),module_number,type,d,a,alpha)
            elif type == "prismatica":
                self.graficar_matriz(right_frame,0,module_number,type,float(data[module_number]),a,alpha)
            else:
                pass

        def slider_callback(value): # Funcion que actualiza la matriz al mover el slider
            if type == "revoluta":
                self.graficar_matriz(right_frame,float(value),module_number,type,d,a,alpha)
            elif type == "prismatica":
                self.graficar_matriz(right_frame,0,module_number,type,float(value),a,alpha)
            else:
                pass

        def update_frame(index): # Funcion para actualizar las imagenes del GIF
                frame = frames[index]
                label_gif.configure(image=frame)
                right_frame.after(100, update_frame, (index + 1) % len(frames))

        # ------------------------------------------------- Crear la interfaz del respectivo modulo -----------------------------------------------------------------
        # Crear contenedor principal
        module_frame = ttk.Frame(parent_frame)
        module_frame.pack(fill=tk.X, padx=10, pady=3)  # Ajuste del padding vertical

        # Crear un frame para contener los 3 subcontenedores
        container_frame = ttk.Frame(module_frame)
        container_frame.pack(fill=tk.X)

        # Frame para los sliders en el lado izquierdo
        left_frame = ttk.Frame(container_frame)
        left_frame.pack(side="left", padx=5, pady=2)

        # Crear los sliders
        slider = tk.Scale(left_frame, from_=initial_min, to=initial_max, orient='horizontal', label=f"Joint {module_number}", resolution=0.001)
        slider.pack(pady=2) 
        self.sliders.append(slider)
        slider.config(command=slider_callback) # El slider se enlaza con la funcion que actualiza las matrices

        # Entrada de valores en las cajas de texto
        value_entry = ttk.Entry(left_frame, width=10)
        value_entry.pack(pady=2)
        value_entry.insert(0, str(round(initial_value, 3)))
        slider.set(initial_value)
        slider.bind("<Motion>", update_value_from_slider)
        value_entry.bind("<FocusOut>", update_slider_from_entry)
        value_entry.bind("<Return>", on_enter_pressed)

        # Limites de los sliders
        limits_frame = ttk.Frame(left_frame)
        limits_frame.pack(padx=3, pady=2)  # Reducir espacio entre elementos
        min_label = ttk.Label(limits_frame, text=f"Min:")
        min_label.pack(anchor=tk.W)
        min_entry = ttk.Entry(limits_frame, width=10)
        min_entry.insert(0, str(initial_min))
        min_entry.pack(anchor=tk.W)
        max_label = ttk.Label(limits_frame, text=f"Max:")
        max_label.pack(anchor=tk.W, pady=(2, 0))
        max_entry = ttk.Entry(limits_frame, width=10)
        max_entry.insert(0, str(initial_max))
        max_entry.pack(anchor=tk.W)
        min_entry.bind("<Return>", lambda e: adjust_slider())
        max_entry.bind("<Return>", lambda e: adjust_slider())

        # Frame para los botones en el centro
        center_frame = ttk.Frame(container_frame)
        center_frame.pack(side="left", padx=5, pady=2)  # Separación del centro

        # Botones de la interfaz
        adjust_button = ttk.Button(center_frame, text=f"Ajustar límites Joint {module_number}", command=adjust_slider)
        adjust_button.pack(pady=2) # Boton para definir limite
        reset_button = ttk.Button(center_frame, text=f"Restablecer valores predeterminados Joint {module_number}", command=reset_defaults)
        reset_button.pack(pady=2) # Boton para reestablecer limites
        if type != "efector":
            matriz_button = ttk.Button(center_frame, text=f"Actualizar matriz XYZ{module_number}", command=actualizar_matriz)
            matriz_button.pack(pady=2) # Boton para actualizar la matriz de la articulacion (excepto por el efecto final, cuya matriz es igual a la de la ultima articulacion)

        # Añadir label para mostrar los valores leídos del Arduino
        label = ttk.Label(center_frame, text=f"Valor medido en Joint {module_number}: 0.000")
        label.pack(pady=2)
        self.labels.append(label)

        # Frame para la matriz en el lado derecho
        right_frame = ttk.Frame(container_frame)
        right_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)  # Centrar y expandir el contenedor

        # Graficar la matriz inicialmente con valor 0
        if type == "revoluta":
            self.graficar_matriz(right_frame,initial_value,module_number,type,d,a,alpha)
        elif type == "prismatica":
            self.graficar_matriz(right_frame,0,module_number,type,initial_value,a,alpha)
        else: # Si el modulo es del efector final, se lee la ruta del gif y se grafica en el espacio donde iria la matriz
            script_dir = os.path.dirname(__file__)  
            path = os.path.join(script_dir, 'wow.gif')
            gif = Image.open(path)
            frames = [ImageTk.PhotoImage(frame.copy()) for frame in ImageSequence.Iterator(gif)]
            # Crear un Label para mostrar el GIF
            label_gif = tk.Label(right_frame)
            label_gif.pack()
            update_frame(0)
            pass

def ros_spin_thread(): # Debido a que tkinter y las comunicaciones de ROS no pueden iniciarse a la vez, se crea un hilo cuya tarea es la comunicacion de ROS
    global node
    rclpy.spin(node)

# ----------------------------------------------------------- MAIN ------------------------------------------------------------------------------------------
def main(args=None): # Funcion principal del programa, se crea el nodo, se inician las comunicaciones y la interfaz, donde ambas se correran en loop hasta la finalizacion del progama
    global node
    rclpy.init(args=args)
    root = tk.Tk()
    node = SliderApp(root)
    spin_thread = threading.Thread(target=ros_spin_thread)
    spin_thread.start()
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__": # Llamada a la funcion principal
    main()

# ------------------------------------------------------------------------------------------------------------------------------------------------------------