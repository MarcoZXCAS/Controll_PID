import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# --- FUNZIONE DI SIMULAZIONE DEL SISTEMA (Plant) ---
def simulate_pid(Kp, Ki, Kd, setpoint):
    dt = 0.05
    t_max = 50.0
    t = np.arange(0, t_max, dt)
    y = np.zeros(len(t))
    
    integral = 0
    prev_error = 0
    
    # Variabili del sistema fisico simulato (es. una massa con attrito)
    pos = 0.0
    vel = 0.0
    
    for i in range(1, len(t)):
        error = setpoint - pos
        integral += error * dt
        derivative = (error - prev_error) / dt
        
        # Calcolo dell'uscita del PID
        control = Kp * error + Ki * integral + Kd * derivative
        
        # Dinamica del sistema (Massa-Molla-Smorzatore semplificato)
        # Accelerazione = (Forza - attrito*velocità) / massa
        acc = control - 0.5 * vel 
        vel += acc * dt
        pos += vel * dt
        
        y[i] = pos
        prev_error = error
        
    return t, y

# --- FUNZIONE PER CALCOLARE I PARAMETRI A REGIME ---
def calculate_metrics(t, y, setpoint):
    current_val = y[-1]
    error = setpoint - current_val
    
    # Calcolo tempo di risposta (quando il sistema rientra nel 2% del valore finale stabilizzato)
    band = max(abs(current_val * 0.02), 0.01)
    response_time = "Mai"
    
    # Se l'errore a regime è altissimo o oscilla all'infinito, lo consideriamo non stabilizzato
    if abs(y[-1] - y[-2]) > 0.01:
        return current_val, error, "Oscillante / Instabile"

    for i in range(len(y)-1, -1, -1):
        if abs(y[i] - current_val) > band:
            if i < len(y) - 1:
                response_time = f"{t[i+1]:.2f}"
            break
    else:
        response_time = "0.00"
        
    return current_val, error, response_time

# --- AGGIORNAMENTO GRAFICO E DATI (Callback in tempo reale) ---
def update_plot(*args):
    try:
        # Legge i valori dalle caselle di testo
        Kp = float(sv_kp.get())
        Ki = float(sv_ki.get())
        Kd = float(sv_kd.get())
        setpoint = float(sv_setpoint.get())
    except ValueError:
        # Se l'utente sta ancora digitando o c'è un campo vuoto, ignora l'errore e aspetta
        return

    # Esegue la simulazione
    t, y = simulate_pid(Kp, Ki, Kd, setpoint)
    
    # Calcola i dati a regime
    current_val, error, response_time = calculate_metrics(t, y, setpoint)
    
    # Aggiorna le etichette dell'interfaccia a sinistra
    lbl_current_val.config(text=f"{current_val:.3f}")
    lbl_error.config(text=f"{error:.3f}")
    lbl_response_time.config(text=f"{response_time} s")
    
    # Aggiorna il grafico
    ax.clear()
    ax.plot(t, y, label="Current Value (Risposta)", color='b', linewidth=2)
    ax.axhline(setpoint, color='r', linestyle='--', label="Setpoint")
    ax.set_title("Risposta nel Tempo del Sistema con PID")
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Ampiezza")
    ax.grid(True)
    ax.legend()
    
    # Ridisegna il canvas
    canvas.draw()

# --- CONFIGURAZIONE INTERFACCIA GRAFICA (Tkinter) ---
root = tk.Tk()
root.title("Simulatore PID in Tempo Reale")
root.geometry("900x600")

# Frame Principale
main_frame = ttk.Frame(root, padding="10")
main_frame.pack(fill=tk.BOTH, expand=True)

# 1. PARTE CENTRALE: Grafico Matplotlib
plot_frame = ttk.Frame(main_frame)
plot_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

fig, ax = plt.subplots(figsize=(8, 4))
canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# 2. PARTE INFERIORE: Controlli e Statistiche
bottom_frame = ttk.Frame(main_frame)
bottom_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=10)

# 2a. A Sinistra: Setpoint e Risultati a Regime
left_frame = ttk.LabelFrame(bottom_frame, text="Parametri di Sistema", padding="10")
left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

# Variabili dinamiche
sv_setpoint = tk.StringVar(value="10.0")
sv_setpoint.trace_add("write", update_plot)

# Layout Griglia per la parte sinistra
ttk.Label(left_frame, text="Setpoint (Destinazione):").grid(row=0, column=0, sticky=tk.W, pady=2)
ttk.Entry(left_frame, textvariable=sv_setpoint, width=10).grid(row=0, column=1, sticky=tk.W, pady=2)

ttk.Label(left_frame, text="Current Value (a regime):").grid(row=1, column=0, sticky=tk.W, pady=2)
lbl_current_val = ttk.Label(left_frame, text="0.000", font=("Arial", 10, "bold"))
lbl_current_val.grid(row=1, column=1, sticky=tk.W, pady=2)

ttk.Label(left_frame, text="Error (Differenza):").grid(row=2, column=0, sticky=tk.W, pady=2)
lbl_error = ttk.Label(left_frame, text="0.000", font=("Arial", 10, "bold"))
lbl_error.grid(row=2, column=1, sticky=tk.W, pady=2)

ttk.Label(left_frame, text="Response Time:").grid(row=3, column=0, sticky=tk.W, pady=2)
lbl_response_time = ttk.Label(left_frame, text="0.00 s", font=("Arial", 10, "bold"))
lbl_response_time.grid(row=3, column=1, sticky=tk.W, pady=2)


# 2b. A Destra: Inserimento Dati PID
right_frame = ttk.LabelFrame(bottom_frame, text="Parametri PID", padding="10")
right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

# Variabili dinamiche (scatenano l'aggiornamento quando cambiano)
sv_kp = tk.StringVar(value="1.5")
sv_ki = tk.StringVar(value="0.1")
sv_kd = tk.StringVar(value="0.5")

sv_kp.trace_add("write", update_plot)
sv_ki.trace_add("write", update_plot)
sv_kd.trace_add("write", update_plot)

# Layout Griglia per la parte destra
ttk.Label(right_frame, text="Kp (Proporzionale):").grid(row=0, column=0, sticky=tk.E, pady=5)
ttk.Entry(right_frame, textvariable=sv_kp, width=10).grid(row=0, column=1, padx=5, pady=5)

ttk.Label(right_frame, text="Ki (Integrale):").grid(row=1, column=0, sticky=tk.E, pady=5)
ttk.Entry(right_frame, textvariable=sv_ki, width=10).grid(row=1, column=1, padx=5, pady=5)

ttk.Label(right_frame, text="Kd (Derivativo):").grid(row=2, column=0, sticky=tk.E, pady=5)
ttk.Entry(right_frame, textvariable=sv_kd, width=10).grid(row=2, column=1, padx=5, pady=5)

# Esegui il primo aggiornamento manuale per popolare il grafico all'avvio
update_plot()

# Avvia l'interfaccia grafica
root.mainloop()