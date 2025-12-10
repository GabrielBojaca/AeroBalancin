import tkinter as tk
from tkinter import ttk
import serial
import threading
import time
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ---------------------------
# CONFIGURAR PUERTO SERIAL
# ---------------------------
SERIAL_PORT = "COM3"      # <--- CAMBIAR A TU PUERTO
BAUDRATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

running = True

# ---------------------------
# VARIABLES PARA PLOTTING
# ---------------------------
max_points = 1000        # tiempo visible en el gráfico (~300 muestras)
data_t = []
data_angle = []
data_ref = []
data_vel = []
data_pwm = []

# ---------------------------
# FUNCIONES PARA MANDAR COMANDOS
# ---------------------------
def enviar_comando(cmd):
    ser.write((str(cmd) + "\n").encode())

def set_setpoint():
    try:
        sp = float(entry_setpoint.get())
        enviar_comando(int(sp))
    except:
        pass

def activar_run():
    enviar_comando(9999)

def enviar_pid():
    enviar_comando(350)

def enviar_hinf():
    enviar_comando(440)

def enviar_smc():
    enviar_comando(550)
def enviar_mrac():
    enviar_comando(650)

def enviar_aprox():
    enviar_comando(250)

def apagar_control():
    enviar_comando(0)

# ---------------------------
# LECTURA DEL SERIAL
# ---------------------------
def procesar_serial(linea):
    linea = linea.strip()

    # Ahora incluyo Vel y PWM
    claves = ["Ref", "Ang", "Vel", "PWM", "MODO_ACT", "MODO_CONT", "RUN", "APROX"]
    valores = {k: "---" for k in claves}

    partes = linea.split()

    # Extraer valores del texto del ESP
    for i, p in enumerate(partes):
        for clave in claves:
            if p.startswith(clave + ":"):
                try:
                    valores[clave] = partes[i+1] if partes[i+1] != "" else "---"
                except:
                    valores[clave] = "---"

    # Mostrar solo los que quieres
    texto_serial.set(
        f"Ref: {valores['Ref']}  "
        f"Ang: {valores['Ang']}  "
        f"Vel: {valores['Vel']}  "
        f"PWM: {valores['PWM']}  "
        f"Modo Act: {valores['MODO_ACT']}  "
        f"Modo Cont: {valores['MODO_CONT']}  "
        f"Run: {valores['RUN']}  "
        f"Aprox: {valores['APROX']}"
    )
def leer_serial():
    global data_t, data_angle, data_ref, data_vel, data_pwm

    while running:
        try:
            linea = ser.readline().decode().strip()
            if not linea:
                continue

            procesar_serial(linea)
            partes = linea.split()

            # Comprobamos que existan los campos
            if all(k in partes for k in ["Ref:", "Ang:", "Vel:", "PWM:"]):
                try:
                    idx_ref = partes.index("Ref:") + 1
                    idx_ang = partes.index("Ang:") + 1
                    idx_vel = partes.index("Vel:") + 1
                    idx_pwm = partes.index("PWM:") + 1

                    ref = float(partes[idx_ref])
                    ang = float(partes[idx_ang])
                    vel = float(partes[idx_vel])
                    pwm = float(partes[idx_pwm])

                    # Guardar tiempo y valores
                    t = time.time()

                    data_t.append(t)
                    data_angle.append(ang)
                    data_ref.append(ref)
                    data_vel.append(vel)
                    data_pwm.append(pwm)

                    # Limitar tamaño
                    if len(data_t) > max_points:
                        data_t = data_t[-max_points:]
                        data_angle = data_angle[-max_points:]
                        data_ref = data_ref[-max_points:]
                        data_vel = data_vel[-max_points:]
                        data_pwm = data_pwm[-max_points:]

                except:
                    pass

        except:
            pass

# ---------------------------
# ACTUALIZAR GRÁFICO
# ---------------------------
def actualizar_grafico():
    if len(data_t) > 5:

        # Normalizar tiempo
        t0 = data_t[0]
        t_norm = [ti - t0 for ti in data_t]

        # --- SINCRONIZAR TAMAÑOS ---
        min_len = min(
            len(t_norm),
            len(data_angle),
            len(data_ref),
            len(data_vel),
            len(data_pwm)
        )

        t_norm = t_norm[-min_len:]
        ang = data_angle[-min_len:]
        ref = data_ref[-min_len:]
        vel = data_vel[-min_len:]
        pwm = data_pwm[-min_len:]

        # ---------------------------------
        # GRAFICA SUPERIOR (Ángulo / Ref)
        # ---------------------------------
        ax1.clear()

        ax1.plot(t_norm, ang, label="Ángulo", linewidth=2)
        ax1.plot(t_norm, ref, label="Referencia", linewidth=2)

        # Líneas horizontales
        ax1.axhline(y=0, color='black', linestyle='--', linewidth=1)
        ax1.axhline(y=180, color='black', linestyle='--', linewidth=1)

        ax1.set_ylim(0, 180)
        ax1.set_ylabel("Ángulo (°)")
        ax1.legend(loc="upper right")
        ax1.grid(True)

        # ---------------------------------
        # GRAFICA INFERIOR (Velocidad / PWM)
        # ---------------------------------
        ax2.clear()

        ax2.plot(t_norm, vel, label="Velocidad ang.", linewidth=2)
        ax2.plot(t_norm, pwm, label="PWM", linewidth=2)

        ax2.set_xlabel("Tiempo (s)")
        ax2.set_ylabel("Vel / PWM")
        ax2.legend(loc="upper right")
        ax2.grid(True)

        canvas.draw()

    root.after(50, actualizar_grafico)

# -------------------------------
# AQUÍ PEGAS LA FUNCIÓN COMPLETA
# -------------------------------



# ---------------------------
# INTERFAZ TKINTER
# ---------------------------
root = tk.Tk()
root.title("Panel de Control ESP32")
root.geometry("1200x650")

# ----------------------------------------------------
# FRAME PRINCIPAL: IZQUIERDA (gráfica) + DERECHA (botones)
# ----------------------------------------------------
main_frame = tk.Frame(root)
main_frame.pack(fill="both", expand=True)

# ----------------------------------------------------
# IZQUIERDA: GRÁFICA
# ----------------------------------------------------
graph_frame = tk.Frame(main_frame, bg="white")
graph_frame.pack(side="left", fill="both", expand=True)

fig = Figure(figsize=(6, 6), dpi=100)

ax1 = fig.add_subplot(211)   # Gráfica superior (ángulo y referencia)
ax2 = fig.add_subplot(212)   # Gráfica inferior (velocidad y PWM)

canvas = FigureCanvasTkAgg(fig, master=graph_frame)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack(fill="both", expand=True, padx=10, pady=10)

# ----------------------------------------------------
# DERECHA: BOTONES
# ----------------------------------------------------
control_frame = tk.Frame(main_frame)
control_frame.pack(side="right", fill="y", padx=20, pady=20)

# Centrado vertical
for i in range(4):
    control_frame.grid_rowconfigure(i, weight=1)

# --- Fila 0 ---
ttk.Button(control_frame, text="RUN", command=activar_run).grid(row=0, column=0, padx=5, pady=5)
ttk.Button(control_frame, text="OFF", command=apagar_control).grid(row=0, column=1, padx=5, pady=5)
ttk.Button(control_frame, text="APROX", command=enviar_aprox).grid(row=0, column=2, padx=5, pady=5)

# --- Fila 1 ---
ttk.Button(control_frame, text="PI_D", command=enviar_pid).grid(row=1, column=0, padx=5, pady=5)
ttk.Button(control_frame, text="H∞", command=enviar_hinf).grid(row=1, column=1, padx=5, pady=5)
ttk.Button(control_frame, text="SMC", command=enviar_smc).grid(row=1, column=2, padx=5, pady=5)
ttk.Button(control_frame, text="MRAC", command=enviar_mrac).grid(row=1, column=3, padx=5, pady=5)

# --- Fila 2: SETPOINT ---
ttk.Label(control_frame, text="Setpoint: ").grid(row=2, column=0, padx=5, pady=5)
entry_setpoint = ttk.Entry(control_frame, width=6)
entry_setpoint.grid(row=2, column=1, padx=5, pady=5)
ttk.Button(control_frame, text="Enviar", command=set_setpoint).grid(row=2, column=2, padx=5, pady=5)

# ----------------------------------------------------
# ABAJO: DATOS DEL ESP32 (más pequeño)
# ----------------------------------------------------
frame_data = ttk.LabelFrame(root, text="Datos del ESP32")
frame_data.pack(fill="x", padx=10, pady=5)

texto_serial = tk.StringVar()
texto_serial.set("Esperando datos...")

label_serial = ttk.Label(frame_data, textvariable=texto_serial, font=("Consolas", 10))
label_serial.pack(padx=5, pady=5)

# ----------------------------------------------------
# HILO SERIAL
# ----------------------------------------------------
hilo = threading.Thread(target=leer_serial, daemon=True)
hilo.start()

# ----------------------------------------------------
# INICIAR GRAFICO
# ----------------------------------------------------
actualizar_grafico()

# Cerrar ventana
def cerrar():
    global running
    running = False
    ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", cerrar)
root.mainloop()
