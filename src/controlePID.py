import serial
import time
import tkinter as tk
from tkinter import messagebox

# === CONFIGURATION ===
PORT = "COM3"  # à adapter
BAUD = 115200

fenetre = tk.Tk()

# === Connexion série ===
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
except Exception as e:
    messagebox.showerror("Erreur Série", f"Impossible d'ouvrir {PORT} : {e}")
    exit()

# === Choix alpha ===
alpha_values = [f"{a:.2f}" for a in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]]
alpha_pos_var = tk.StringVar()
alpha_pos_var.set("0.6")
alpha_vit_var = tk.StringVar()
alpha_vit_var.set("0.4")
forceAccoup_var = tk.StringVar()
forceAccoup_var.set("0.5")

# === Fonction d'envoi des valeurs PID ===
def envoyer_pid():
    kp_p = slider_kp_pos.get()
    ki_p = slider_ki_pos.get()
    kd_p = slider_kd_pos.get()
    kp_v = slider_kp_vit.get()
    ki_v = slider_ki_vit.get()
    kd_v = slider_kd_vit.get()
    alpha_p = float(alpha_pos_var.get())
    alpha_v = float(alpha_vit_var.get())
    forceAccoup = float(forceAccoup_var.get())

    commande = f"PID:{kp_p:.3f},{ki_p:.3f},{kd_p:.3f},{kp_v:.3f},{ki_v:.3f},{kd_v:.3f},{alpha_p:.2f},{alpha_v:.2f}\n"
    try:
        ser.write(commande.encode())
        print(f"[TX] → {commande.strip()}")
        retour = ser.readline().decode().strip()
        label_reponse.config(text=f"[ESP] {retour}")
    except Exception as e:
        label_reponse.config(text=f"Erreur d'envoi: {e}")

# === Interface graphique ===
fenetre.title("Contrôle PID + Alpha - ESP32")

def ajout_section_titre(titre):
    tk.Label(fenetre, text=titre, font=("Arial", 12, "bold")).pack(pady=(10, 0))

def ajout_slider(slider):
    slider.pack()

# PID Position
ajout_section_titre("PID Position")
slider_kp_pos = tk.Scale(fenetre, from_=0.0, to=10.0, resolution=0.01, label="Kp (pos)", orient=tk.HORIZONTAL, length=300)
slider_ki_pos = tk.Scale(fenetre, from_=0.0, to=1.0, resolution=0.001, label="Ki (pos)", orient=tk.HORIZONTAL, length=300)
slider_kd_pos = tk.Scale(fenetre, from_=0.0, to=300.0, resolution=0.1, label="Kd (pos)", orient=tk.HORIZONTAL, length=300)
slider_kp_pos.set(0.65)
slider_ki_pos.set(0.002)
slider_kd_pos.set(36.9)
for s in (slider_kp_pos, slider_ki_pos, slider_kd_pos): ajout_slider(s)

# Alpha Position
tk.Label(fenetre, text="Alpha (position):").pack()
menu_alpha_pos = tk.OptionMenu(fenetre, alpha_pos_var, *alpha_values)
menu_alpha_pos.pack()

# PID Vitesse
ajout_section_titre("PID Vitesse")
slider_kp_vit = tk.Scale(fenetre, from_=0.0, to=10.0, resolution=0.01, label="Kp (vit)", orient=tk.HORIZONTAL, length=300)
slider_ki_vit = tk.Scale(fenetre, from_=0.0, to=1.0, resolution=0.001, label="Ki (vit)", orient=tk.HORIZONTAL, length=300)
slider_kd_vit = tk.Scale(fenetre, from_=0.0, to=300.0, resolution=0.1, label="Kd (vit)", orient=tk.HORIZONTAL, length=300)
slider_kp_vit.set(1.21)
slider_ki_vit.set(0.004)
slider_kd_vit.set(123.1)
for s in (slider_kp_vit, slider_ki_vit, slider_kd_vit): ajout_slider(s)

# Alpha Vitesse
tk.Label(fenetre, text="Alpha (vitesse):").pack()
menu_alpha_vit = tk.OptionMenu(fenetre, alpha_vit_var, *alpha_values)
menu_alpha_vit.pack()

# Force Accoup
tk.Label(fenetre, text="Force Accoup:").pack()
menu_forceAccoup = tk.OptionMenu(fenetre, forceAccoup_var, *alpha_values)
menu_forceAccoup.pack()

# Bouton d'envoi
btn_envoyer = tk.Button(fenetre, text="Envoyer PID + Alpha", command=envoyer_pid, bg="green", fg="white", font=("Arial", 11, "bold"))
btn_envoyer.pack(pady=20)

# Réponse ESP
label_reponse = tk.Label(fenetre, text="[ESP] En attente...", fg="blue")
label_reponse.pack(pady=10)

fenetre.mainloop()
ser.close()
