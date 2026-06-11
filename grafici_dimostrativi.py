import matplotlib.pyplot as plt
import numpy as np

# ==========================================
# PARAMETRI CONDIVISI
# ==========================================
x = np.linspace(0, 15, 1000)  # Esteso a 15 per vedere meglio la convergenza
t_0 = 5                        # Il punto di attivazione iniziale (x=5)
y_0 = 0                        # Valore iniziale prima dell'attivazione

# ==========================================
# FINESTRA 1: Scalino vs Sigmoide Esponenziale
# ==========================================
plt.figure(num="Curva di Reazione", figsize=(8, 5))

# Parametri specifici per la sigmoide (curva di carica)
C_0 = 1.0   # Valore asintotico finale
T = 1.5     # Costante di tempo (regola la velocita di salita)
t_i = 0.2   # Ritardo aggiuntivo rispetto a t_0

# Scalino: 0 prima di t_0, 1 dopo
y_scalino = np.where(x >= t_0, 1.0, 0.0)

# Sigmoide attiva solo per x >= (t_i + t_0)
t_attivazione_sig = t_i + t_0
y_sigmoide = np.where(x >= t_attivazione_sig, y_0 + (C_0 - y_0) * (1 - np.exp(-(x - t_attivazione_sig) / T)), y_0)

# Disegno Grafico 1
plt.plot(x, y_scalino, color='black', linewidth=1, linestyle='-', label='Scalino')
plt.plot(x, y_sigmoide, color='red', linestyle='-', label='Sigmoide')

plt.title('Curva di Reazione', fontsize=10, fontweight='bold')
plt.xlabel('x')
plt.ylabel('f(x)')
plt.legend()


# ==========================================
# FINESTRA 2: Coseno con Impulso Precedente
# ==========================================
plt.figure(num="Guadagno Critico", figsize=(8, 5))

# Parametri specifici per il coseno e l'impulso
A = 1.5              # Ampiezza dell'oscillazione e dell'impulso
w = 8.0              # Frequenza angolare (omega)
phi = np.pi/2        # Fase
durata_impulso = 0.2 # Durata del breve impulso iniziale
t_i = 0.1            # Ritardo aggiuntivo rispetto a t_0

# Il coseno partira' esattamente dove finisce l'impulso
t_inizio_coseno = t_0 + t_i

# 1. IMPULSO RETTANGOLARE BREVE: attivo solo tra t_0 e t_inizio_coseno
# Usiamo np.logical_and per definire la finestra temporale dell'impulso
y_impulso = np.where(np.logical_and(x >= t_0, x < t_inizio_coseno), A, 0.0)

# 2. COSENO: attivo solo dopo la fine dell'impulso (x >= t_inizio_coseno)
# Modifichiamo l'argomento per far si che parta da 0 nella nuova origine temporale
y_coseno = np.where(x >= t_inizio_coseno, y_0 + A * np.cos(w * (x - t_inizio_coseno) - phi), 0.0)

# Disegno Grafico 2
plt.plot(x, y_impulso, color='black', linewidth=1, linestyle='-', label='Impulso')
plt.plot(x, y_coseno, color='red', linestyle='-', label='Coseno')

plt.title('Guadagno Critico', fontsize=10, fontweight='bold')
plt.xlabel('x')
plt.ylabel('f(x)')
plt.legend()

# ==========================================
# MOSTRA I GRAFICI
# ==========================================
plt.show()
