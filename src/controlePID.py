import serial
import time
import tkinter as tk
from tkinter import messagebox

# === CONFIGURATION ===
PORT = "COM3"  
BAUD = 115200

fenetre = tk.Tk()
fenetre.title("Contrôle PID - ESP32")

# === Connexion série ===
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
except Exception as e:
    messagebox.showerror("Erreur Série", f"Impossible d'ouvrir {PORT} : {e}")
    exit()

# === Variables globales ===
alpha_values = [f"{a:.2f}" for a in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]]
alpha_pos_var = tk.StringVar(value="0.5")
alpha_vit_var = tk.StringVar(value="0.5")
mode_flagActivLissagePos = tk.BooleanVar(value=True)
mode_flagActivLissageVit = tk.BooleanVar(value=True)

# === Fonction d'envoi des valeurs PID ===
def envoyer_pid():
    kp_p = slider_kp_pos.get()
    ki_p = slider_ki_pos.get()
    kd_p = slider_kd_pos.get()
    kp_v = slider_kp_vit.get()
    ki_v = slider_ki_vit.get()
    kd_v = slider_kd_vit.get()
    kp_i = slider_kp_inclin.get()
    ki_i = slider_ki_inclin.get()
    kd_i = slider_kd_inclin.get()
    alpha_p = float(alpha_pos_var.get())
    alpha_v = float(alpha_vit_var.get())
    lissage_pos = int(mode_flagActivLissagePos.get())
    lissage_vit = int(mode_flagActivLissageVit.get())
    nb_vals_pos = slider_nb_valeurs_pos.get()
    nb_vals_vit = slider_nb_valeurs_vit.get()
    sliderA = sliderA_val.get()
    sliderB = sliderB_val.get() 
    sliderC = sliderC_val.get()
    sliderD = sliderD_val.get()
    factIncliaison = slider_Inclinaison.get()

    commande = f"PID:{kp_p:.3f},{ki_p:.3f},{kd_p:.3f},{kp_v:.3f},{ki_v:.3f},{kd_v:.3f},{kp_i:.3f},{ki_i:.3f},{kd_i:.3f},{alpha_p:.2f},{alpha_v:.2f},{lissage_pos},{lissage_vit},{nb_vals_pos},{nb_vals_vit},{sliderA},{sliderB},{sliderC},{sliderD},{factIncliaison}\n"

    try:
        ser.write(commande.encode())
        print(f"[TX] → {commande.strip()}")
        retour = ser.readline().decode().strip()
        label_reponse.config(text=f"[ESP] {retour}")
    except Exception as e:
        label_reponse.config(text=f"Erreur d'envoi: {e}")

# === Layout sliders PID (côte à côte) ===
frame_pid_triple = tk.Frame(fenetre)
frame_pid_triple.pack(pady=10)

frame_pid_dual = tk.Frame(fenetre)
frame_pid_dual.pack(pady=10)

frame_pid_pos = tk.Frame(frame_pid_triple)
frame_pid_pos.pack(side=tk.LEFT, padx=20)

frame_pid_vit = tk.Frame(frame_pid_triple)
frame_pid_vit.pack(side=tk.LEFT, padx=20)

frame_pid_inclin = tk.Frame(frame_pid_triple)
frame_pid_inclin.pack(side=tk.LEFT, padx=20)

frame_alpha_pos = tk.Frame(frame_pid_dual)
frame_alpha_pos.pack(side=tk.LEFT, padx=20)

frame_alpha_vit = tk.Frame(frame_pid_dual)
frame_alpha_vit.pack(side=tk.RIGHT, padx=20)

# --- PID Position ---
tk.Label(frame_pid_pos, text="PID Position", font=("Arial", 12, "bold")).pack()
slider_kp_pos = tk.Scale(frame_pid_pos, from_=0.0, to=10.0, resolution=0.01, label="Kp (pos)", orient=tk.HORIZONTAL, length=250)
slider_ki_pos = tk.Scale(frame_pid_pos, from_=0.0, to=1.0, resolution=0.001, label="Ki (pos)", orient=tk.HORIZONTAL, length=250)
slider_kd_pos = tk.Scale(frame_pid_pos, from_=0.0, to=300.0, resolution=0.1, label="Kd (pos)", orient=tk.HORIZONTAL, length=250)
slider_kp_pos.set(2.06)
slider_ki_pos.set(0.014)
slider_kd_pos.set(134.9)
for s in (slider_kp_pos, slider_ki_pos, slider_kd_pos): s.pack()

# --- PID Vitesse ---
tk.Label(frame_pid_vit, text="PID Vitesse", font=("Arial", 12, "bold")).pack()
slider_kp_vit = tk.Scale(frame_pid_vit, from_=0.0, to=10.0, resolution=0.01, label="Kp (vit)", orient=tk.HORIZONTAL, length=250)
slider_ki_vit = tk.Scale(frame_pid_vit, from_=0.0, to=1.0, resolution=0.001, label="Ki (vit)", orient=tk.HORIZONTAL, length=250)
slider_kd_vit = tk.Scale(frame_pid_vit, from_=0.0, to=300.0, resolution=0.1, label="Kd (vit)", orient=tk.HORIZONTAL, length=250)
slider_kp_vit.set(0.55)
slider_ki_vit.set(0.012)
slider_kd_vit.set(140.4)
for s in (slider_kp_vit, slider_ki_vit, slider_kd_vit): s.pack()

# --- PID Inclinaison ---
tk.Label(frame_pid_inclin, text="PID Inclinaison", font=("Arial", 12, "bold")).pack()
slider_kp_inclin = tk.Scale(frame_pid_inclin, from_=0.0, to=10.0, resolution=0.01, label="Kp (inclinaison)", orient=tk.HORIZONTAL, length=250)
slider_ki_inclin = tk.Scale(frame_pid_inclin, from_=0.0, to=1.0, resolution=0.001, label="Ki (inclinaison)", orient=tk.HORIZONTAL, length=250)
slider_kd_inclin = tk.Scale(frame_pid_inclin, from_=0.0, to=300.0, resolution=0.1, label="Kd (inclinaison)", orient=tk.HORIZONTAL, length=250)
slider_kp_inclin.set(2.06)
slider_ki_inclin.set(0.014)
slider_kd_inclin.set(134.9)
for s in (slider_kp_inclin, slider_ki_inclin, slider_kd_inclin): s.pack()

# === Alpha ===
tk.Label(frame_alpha_pos, text="Alpha (position)").pack()
menu_alpha_pos = tk.OptionMenu(frame_alpha_pos, alpha_pos_var, *alpha_values)
menu_alpha_pos.pack()

tk.Label(frame_alpha_vit, text="Alpha (vitesse)").pack()
menu_alpha_vit = tk.OptionMenu(frame_alpha_vit, alpha_vit_var, *alpha_values)
menu_alpha_vit.pack()

# === Lissage activation checkboxes ===
check_pos = tk.Checkbutton(fenetre, text="Activer le lissage des positions", variable=mode_flagActivLissagePos)
check_pos.pack()
check_vit = tk.Checkbutton(fenetre, text="Activer le lissage des vitesses", variable=mode_flagActivLissageVit)
check_vit.pack()

# === Nombre de valeurs à échantillonner ===
tk.Label(fenetre, text="Nombre de valeurs à échantillonner pour la position :").pack(pady=(10, 0))
slider_nb_valeurs_pos = tk.Scale(fenetre, from_=1, to=10, orient=tk.HORIZONTAL, length=300)
slider_nb_valeurs_pos.set(7)
slider_nb_valeurs_pos.pack()

tk.Label(fenetre, text="Nombre de valeurs à échantillonner pour la vitesse :").pack(pady=(10, 0))
slider_nb_valeurs_vit = tk.Scale(fenetre, from_=1, to=10, orient=tk.HORIZONTAL, length=300)
slider_nb_valeurs_vit.set(3)
slider_nb_valeurs_vit.pack()

# === Layout sliders réglage puissance bobines (côte à côte) ===
frame_pid_dual = tk.Frame(fenetre)
frame_pid_dual.pack(pady=10)

frame_pid_bobCD = tk.Frame(frame_pid_dual)
frame_pid_bobCD.pack(side=tk.LEFT, padx=20)

frame_pid_AB = tk.Frame(frame_pid_dual)
frame_pid_AB.pack(side=tk.LEFT, padx=20)

tk.Label(frame_pid_bobCD, text="C/D", font=("Arial", 12, "bold")).pack()
sliderC_val = tk.Scale(frame_pid_bobCD, from_=-500.0, to=500.0, resolution=10, label="bobine C", orient=tk.HORIZONTAL, length=250)
sliderD_val = tk.Scale(frame_pid_bobCD, from_=-500.0, to=500.0, resolution=10, label="bobine D", orient=tk.HORIZONTAL, length=250)
sliderC_val.set(40)
sliderD_val.set(0)
for s in (sliderC_val, sliderD_val): s.pack()

tk.Label(frame_pid_AB, text="B/A", font=("Arial", 12, "bold")).pack()
sliderB_val = tk.Scale(frame_pid_AB, from_=-500.0, to=500.0, resolution=10, label="bobine B", orient=tk.HORIZONTAL, length=250)
sliderA_val = tk.Scale(frame_pid_AB, from_=-500.0, to=500.0, resolution=10, label="bobine A", orient=tk.HORIZONTAL, length=250)
sliderB_val.set(80)
sliderA_val.set(40)
for s in (sliderB_val, sliderA_val): s.pack()

slider_Inclinaison = tk.Scale(fenetre, from_=0.0, to=5.0, resolution=0.1, label="facteur inclinaison", orient=tk.HORIZONTAL, length=500)
slider_Inclinaison.set(0.8)
slider_Inclinaison.pack(pady=(10, 0))


# === Bouton d'envoi ===
btn_envoyer = tk.Button(fenetre, text="Envoyer PID + Alpha", command=envoyer_pid, bg="green", fg="white", font=("Arial", 11, "bold"))
btn_envoyer.pack(pady=20)

# === Réponse série ===
label_reponse = tk.Label(fenetre, text="[ESP] En attente...", fg="blue")
label_reponse.pack(pady=10)

# === Lancement de l'interface ===
fenetre.mainloop()
ser.close()
