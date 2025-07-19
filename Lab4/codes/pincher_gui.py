import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import rclpy
from pincher_control.control_servo import PincherController
import threading
from tkinter import PhotoImage

# Posiciones predeterminadas
preset_positions = {
    "Pose 1": [512, 512, 512, 512, 512] ,
    "Pose 2": [582, 582, 568, 454, 512],
    "Pose 3": [412, 610, 426, 596, 512],
    "Pose 4": [753, 454, 667, 582, 512],
    "Pose 5": [738, 412, 667, 383, 512]
}

# Iniciar ROS y controlador
rclpy.init()
pincher = PincherController()

# Crear ventana principal
ventana = tk.Tk()
ventana.title("Control de Robot Pincher")
ventana.geometry("600x500")
ventana.configure(bg="#1e1e1e")

# ======= CABECERA =========
header = tk.Frame(ventana, bg="#1e1e1e")
header.pack(pady=10)

# Cargar imagen (opcional)
try:
    img = PhotoImage(file="/home/santiago/ros2_ws/phantom_ws/src/pincher_control/pincher_control/unal_logo.png")
    logo = tk.Label(ventana, image=img, bg="#f0f0f0")
    logo.image = img  # importante para que no se borre
    logo = tk.Label(header, image=img, bg="#1e1e1e")
    logo.grid(row=0, column=0, rowspan=3, padx=10)
except Exception as e:
    print("Error cargando imagen:", e)

titulo = tk.Label(header, text="Robótica 2025-1", font=("Helvetica", 20, "bold"), fg="#00ff88", bg="#1e1e1e")
titulo.grid(row=0, column=1, sticky="w")

subtitulo = tk.Label(header, text="Universidad Nacional de Colombia", font=("Helvetica", 14), fg="#ffffff", bg="#1e1e1e")
subtitulo.grid(row=1, column=1, sticky="w")

autores = tk.Label(header, text="Jorge Emilio Melo Guevara\nJaime Andrés Martín", font=("Helvetica", 10), fg="#cccccc", bg="#1e1e1e")
autores.grid(row=2, column=1, sticky="w")

# ======= BOTONES DE POSICIÓN =========
frame_botones = tk.LabelFrame(ventana, text="Poses Dadas", fg="#00ff88", bg="#2e2e2e", font=("Helvetica", 12, "bold"), padx=10, pady=10)
frame_botones.pack(pady=10, padx=20, fill="x")

def mover_a(posicion):
    def _thread():
        pincher.cambioPos(posicion)
    threading.Thread(target=_thread).start()

for nombre, posicion in preset_positions.items():
    b = tk.Button(frame_botones, text=nombre, width=30, bg="#00ff88", fg="black", command=lambda p=posicion: mover_a(p))
    b.pack(pady=4)

# ======= POSICIÓN PERSONALIZADA =========
frame_personalizado = tk.LabelFrame(ventana, text="Pose Manual" , fg="#00ff88", bg="#2e2e2e", font=("Helvetica", 12, "bold"), padx=10, pady=10)
frame_personalizado.pack(padx=20, fill="x")

entrada = tk.Entry(frame_personalizado, width=50)
entrada.pack(pady=5)
entrada.insert(0, "512,512,512,512,512")

def enviar_personalizado():
    texto = entrada.get()
    try:
        valores = [int(x.strip()) for x in texto.split(',')]
        if len(valores) != 5:
            raise ValueError
        mover_a(valores)
    except:
        messagebox.showerror("Error", "Ingresa 5 números separados por comas")

boton_custom = tk.Button(frame_personalizado, text="Enviar a pose", bg="#00ff88", fg="black", command=enviar_personalizado)
boton_custom.pack(pady=5)

# ======= CIERRE SEGURO =========
def cerrar():
    pincher.terminar()
    ventana.destroy()

ventana.protocol("WM_DELETE_WINDOW", cerrar)
ventana.mainloop()
