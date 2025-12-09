import serial
import time
import csv
import matplotlib.pyplot as plt
import pandas as pd
import re  # Importamos buscador de patrones

# --- CONFIGURACIÓN ---
PUERTO = 'COM3'        # <--- CONFIRMA TU PUERTO
BAUDIOS = 115200
NOMBRE_PRUEBA = 'MRAC' 
DURACION_MAX = 15      

datos_raw = []

print(f"--- INICIANDO PRUEBA: {NOMBRE_PRUEBA} ---")
print(f"Conectando a {PUERTO}...")

try:
    ser = serial.Serial(PUERTO, BAUDIOS, timeout=1)
    ser.reset_input_buffer()
    print("CONECTADO. Esperando datos...")
    print(f"El script buscará patrones como 'Ref:', 'Ang:', etc.")

    inicio = time.time()
    
    while (time.time() - inicio) < DURACION_MAX:
        if ser.in_waiting > 0:
            try:
                linea = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # --- FILTRO INTELIGENTE (REGEX) ---
                # Buscamos el patrón: "Ref: numero", "Ang: numero", etc.
                if "Ref:" in linea and "Ang:" in linea:
                    
                    # 1. Extraer TIEMPO (El primer numero de la linea)
                    match_t = re.search(r'^(\d+)', linea)
                    t_ms = float(match_t.group(1)) if match_t else 0
                    tiempo_s = t_ms / 1000.0 # Convertir a segundos

                    # 2. Extraer REFERENCIA
                    match_ref = re.search(r'Ref:\s*([-.\d]+)', linea)
                    ref = float(match_ref.group(1)) if match_ref else 0

                    # 3. Extraer ANGULO
                    match_ang = re.search(r'Ang:\s*([-.\d]+)', linea)
                    ang = float(match_ang.group(1)) if match_ang else 0

                    # 4. Extraer PWM
                    match_pwm = re.search(r'PWM:\s*([-.\d]+)', linea)
                    pwm = float(match_pwm.group(1)) if match_pwm else 0

                    # 5. Extraer VELOCIDAD (GyroZ)
                    # Nota: GyroZ suele venir en rad/s o deg/s. Asumimos valor directo.
                    match_vel = re.search(r'GyroZ:\s*([-.\d]+)', linea)
                    vel = float(match_vel.group(1)) if match_vel else 0

                    # Guardamos [Tiempo, Ref, Angulo, Velocidad, PWM]
                    fila = [tiempo_s, ref, ang, vel, pwm]
                    datos_raw.append(fila)
                    print(f"Capturado: T={tiempo_s:.2f}, Ref={ref}, Ang={ang}")

            except Exception as e:
                # Si falla una línea rara, la ignoramos
                pass

except KeyboardInterrupt:
    print("\nDetenido por usuario.")
except Exception as e:
    print(f"Error crítico: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()

# --- GRAFICAR ---
print(f"\nSe capturaron {len(datos_raw)} datos válidos.")

if len(datos_raw) < 5:
    print("ERROR: No se capturaron datos. Verifica que el Monitor Serie de VSCode esté CERRADO.")
else:
    # Guardar CSV limpio
    nombre_archivo = f"datos_{NOMBRE_PRUEBA}.csv"
    with open(nombre_archivo, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Tiempo", "Ref", "Angulo", "Velocidad", "PWM"])
        writer.writerows(datos_raw)

    try:
        df = pd.DataFrame(datos_raw, columns=["Tiempo", "Ref", "Angulo", "Velocidad", "PWM"])
        
        # Ajustar tiempo a cero relativo
        if not df.empty:
            df['Tiempo'] = df['Tiempo'] - df['Tiempo'].iloc[0]

        # Configuración de gráfica
        plt.style.use('seaborn-v0_8-whitegrid')
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
        fig.suptitle(f'Resultados Controlador {NOMBRE_PRUEBA}', fontsize=16, fontweight='bold')

        # 1. Posición
        ax1.plot(df['Tiempo'], df['Ref'], 'k--', label='Referencia', linewidth=1.5)
        ax1.plot(df['Tiempo'], df['Angulo'], 'b-', label='Salida Real', linewidth=2)
        ax1.set_ylabel('Ángulo (°)')
        ax1.legend(loc='lower right')
        ax1.grid(True)

        # 2. Velocidad
        ax2.plot(df['Tiempo'], df['Velocidad'], 'r-', linewidth=1.5)
        ax2.set_ylabel('Velocidad (GyroZ)') # Nota: GyroZ puro
        ax2.grid(True)

        # 3. Control
        ax3.plot(df['Tiempo'], df['PWM'], 'g-', linewidth=1.5)
        ax3.set_ylabel('PWM')
        ax3.set_xlabel('Tiempo (s)')
        ax3.grid(True)

        nombre_imagen = f"grafica_{NOMBRE_PRUEBA}.png"
        plt.savefig(nombre_imagen, dpi=300)
        print(f"¡EXITO! Imagen guardada: {nombre_imagen}")
        plt.show()

    except Exception as e:
        print(f"Error al graficar: {e}")