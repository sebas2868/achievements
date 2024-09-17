import customtkinter as ctk
from tkinter import simpledialog, messagebox, ttk
import tkinter as tk
from PIL import Image, ImageTk, ImageSequence
import numpy as np
import som_library
import threading
import csv
import matplotlib.pyplot as plt
import pandas as pd

class WorkOutApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.som_entrenado = 0
        self.mapa = 0
        self.title("WorkOut Manager")
        self.geometry("500x300")
        self.resizable(True, True)
        self.configure(fg_color="#FFC080")  # warm beige background

        self.csv_file_path = "ejercicio_nuevo.csv"  # Set your CSV file path here

        self.create_widgets()
        colores = np.load('colores_16.npy')


    def create_widgets(self):
        # Title
        title = ctk.CTkLabel(self, text="WorkOut Manager", font=("Impact", 20, "bold"), text_color="black", fg_color="#FFC080")
        title.pack(pady=10)

        # Buttons frame
        button_frame = ctk.CTkFrame(self, fg_color="#FFC080")
        button_frame.pack(pady=20)

        # "Ver SOM" Button
        ver_som_button = ctk.CTkButton(button_frame, text="Ver SOM", font=("Times New Roman", 14, "bold"), command=self.ver_som,
                                       fg_color="#8B9467",  # tetradic chromatic color combination
                                       hover_color="#9AAB73")
        ver_som_button.grid(row=0, column=0, padx=10)

        # "Retrain" Button
        retrain_button = ctk.CTkButton(button_frame, text="Retrain", font=("Times New Roman", 14, "bold"), command=self.start_retrain,
                                       fg_color="#73A09A",  # tetradic chromatic color combination
                                       hover_color="#8BC3A5")
        retrain_button.grid(row=0, column=1, padx=10)

        # "Add" Button
        add_button = ctk.CTkButton(button_frame, text="Add", font=("Times New Roman", 14, "bold"), command=self.add_workout,
                                   fg_color="#94678B",  # tetradic chromatic color combination
                                   hover_color="#A09A9A")
        add_button.grid(row=0, column=2, padx=10)

        # "Close" Button
        close_button = ctk.CTkButton(self, text="Close", font=("Lucida Sans Unicode", 14, "bold"), text_color="purple", command=self.destroy,
                                     fg_color="#8B9467",  # tetradic chromatic color combination
                                     hover_color="#9a7b4f")
        close_button.pack(pady=10)
        
        # Set up logo
        logo_image = Image.open("logo1.png")
        logo_image = logo_image.resize((60, 60), Image.LANCZOS)
        self.logo_photo = ctk.CTkImage(light_image=logo_image, size=(100, 100))  # Use CTkImage
        self.logo_label = ctk.CTkLabel(self, image=self.logo_photo, text='')
        self.logo_label.place(x=380, y=140)  # Adjust the position as needed
        
    def ver_som(self):
        
        msg_box = tk.messagebox.askquestion(
                    "Question",
                    "¿Deseas verlo en 3D?",
                    icon="warning",
                )
                
        if msg_box == "yes":
            if self.som_entrenado == 0:
                self.som_entrenado = 1
                df = som_library.cargar_datos()
                self.mapa, self.data_set, self.data_set_1, self.ganadoras = som_library.entrenar_som(df)
                som_library.graficar_som(self.mapa, self.data_set, self.data_set_1)
                som_library.graficar_som_toroide(self.mapa, self.data_set, self.data_set_1)
            else:
                som_library.graficar_som(self.mapa, self.data_set, self.data_set_1)
                som_library.graficar_som_toroide(self.mapa, self.data_set, self.data_set_1)
        else:
            if self.som_entrenado == 0:
                self.som_entrenado = 1
                df = som_library.cargar_datos()
                self.mapa, self.data_set, self.data_set_1, self.ganadoras = som_library.entrenar_som(df)
                som_library.graficar_som(self.mapa, self.data_set, self.data_set_1)
                plt.show()
            else:
                som_library.graficar_som(self.mapa, self.data_set, self.data_set_1)
                plt.show()
                    
                    
        
     

    def start_retrain(self):
        nEpocas = simpledialog.askstring("Input", "Especifica la cantidad de epocas:")
        if nEpocas is not None:
            try:
                nEpocas = int(nEpocas)
                self.show_progress_window(nEpocas)
                retrain_thread = threading.Thread(target=self.retrain, args=(nEpocas,))
                retrain_thread.start()
            except ValueError:
                messagebox.showerror("Error", "Por favor, introduce un número válido de épocas.")

    def show_progress_window(self, nEpocas):
        self.progress_window = tk.Toplevel(self)
        self.progress_window.title("Entrenando...")
        self.progress_window.geometry("500x500")
        self.progress_window.configure(bg="#FFC080")

        # Title
        title = ctk.CTkLabel(self.progress_window, text="Entrenando...", font=("Impact", 20, "bold"), text_color="black", fg_color="#FFC080")
        title.pack(pady=10)

        # Load GIF
        self.gif_image = Image.open("155c3fd369def67777217d621f900fa7.gif")
        self.frames = [ImageTk.PhotoImage(frame.copy().convert("RGBA")) for frame in ImageSequence.Iterator(self.gif_image)]
        
        self.gif_label = tk.Label(self.progress_window, bg="#FFC080")
        self.gif_label.pack(pady=10)
        
        self.progress_bar = ttk.Progressbar(self.progress_window, orient="horizontal", length=400, mode="determinate")
        self.progress_bar.pack(pady=20)

        self.animate_gif(0)

        self.progress_bar['value'] = 0
        self.progress_bar['maximum'] = nEpocas

    def animate_gif(self, frame_index):
        self.gif_label.config(image=self.frames[frame_index])
        frame_index = (frame_index + 1) % len(self.frames)
        self.progress_window.after(100, self.animate_gif, frame_index)

    def retrain(self, nEpocas):
        df = som_library.cargar_datos()

        for epoch in range(nEpocas):
            self.mapa, self.data_set, self.data_set_1, self.ganadoras = som_library.entrenar_som(df, 1)  # Train one epoch at a time
            self.progress_bar['value'] += 1
            self.progress_window.update_idletasks()

        self.progress_window.destroy()
        messagebox.showinfo("Info", "SOM reentrenado satisfactoriamente.")
    
    def add_workout(self):
        if self.som_entrenado == 0:
            self.som_entrenado = 1
            df = som_library.cargar_datos()
            self.mapa, self.data_set, self.data_set_1, self.ganadoras  = som_library.entrenar_som(df)

        preguntas = [
            "¿Este ejercicio es Cardio?",
            "¿Este ejercicio es de Peso Corporal?",
            "¿Este ejercicio es de Alta Intensidad?",
            "¿Este ejercicio es de Baja Intensidad?",
            "¿Este ejercicio es para Hipertrofia?",
            "¿Este ejercicio es para Fuerza?",
            "¿Este ejercicio es Compuesto?",
            "¿Este ejercicio es Multiarticular?",
            "¿Este ejercicio es Aislado?",
            "¿Este ejercicio trabaja Pecho?",
            "¿Este ejercicio trabaja Tren Superior?",
            "¿Este ejercicio es de Empuje?",
            "¿Este ejercicio es de Jale?",
            "¿Este ejercicio trabaja Core?",
            "¿Este ejercicio trabaja Pierna?"
        ]

        nuevo_objeto = np.zeros(len(preguntas), dtype=int)
        new_workout = simpledialog.askstring("Input", "Digita el nombre del ejercicio:")
        
        if new_workout:
            for i in range(len(preguntas)):
                msg_box = tk.messagebox.askquestion(
                    "Question",
                    preguntas[i],
                    icon="info",
                )
                
                if msg_box == "yes":
                    nuevo_objeto[i] = 1
                else:
                    nuevo_objeto[i] = 0
            
            distancias_ordenadas = som_library.agregar_objeto(self.mapa,nuevo_objeto, self.ganadoras,self.data_set)
            # Prepara el mensaje con los 5 ejercicios más cercanos
            mensaje = f"5 ejercicios más cercanos a {new_workout}:\n"
            for nombre, distancia in distancias_ordenadas[:5]:
                mensaje += f" - {nombre} a una distancia de {distancia:.4f}\n"

            # Muestra la información en un solo cuadro de diálogo
            messagebox.showinfo("5 ejercicios más cercanos", mensaje)

            # Agregar a CSV
            self.append_to_csv(new_workout, nuevo_objeto[:])
            messagebox.showinfo("Info", f"Nuevo ejercicio '{new_workout}' agregado a la base de datos.")
            df = som_library.cargar_datos()
            self.mapa, self.data_set, self.data_set_1,self.ganadoras  = som_library.entrenar_som(df)
            som_library.graficar_som(self.mapa, self.data_set, self.data_set_1)
            plt.show()

        else:
            messagebox.showinfo("Info", "Ningún ejercicio agregado.")

    def append_to_csv(self, workout_name, characteristics):
    # Leer los datos existentes del CSV
        df = pd.read_csv(self.csv_file_path)
     # Crear un DataFrame con la nueva columna
        new_data = pd.DataFrame({workout_name: characteristics})
    # Concatenar el nuevo DataFrame con el DataFrame existente
        df = pd.concat([df, new_data], axis=1)
    # Escribir el DataFrame actualizado de vuelta al archivo CSV
        df.to_csv(self.csv_file_path, index=False)

    def show_similar_exercises(self, similares):
        popup = tk.Toplevel(self)
        popup.wm_title("Ejercicios Similares")
        label = tk.Label(popup, text="Top 5 ejercicios similares:")
        label.pack(side="top", fill="x", pady=10)
        for ejercicio, similitud in similares:
            msg = tk.Message(popup, text=f"{ejercicio} (Similitud: {similitud:.2f})")
            msg.pack()
        button = tk.Button(popup, text="Okay", command=popup.destroy)
        button.pack()

if __name__ == "__main__":
    app = WorkOutApp()
    app.mainloop()
