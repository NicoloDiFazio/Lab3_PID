import numpy as np
import matplotlib.pyplot as plt
from iminuit import Minuit
from pathlib import Path

"""
Questo file serve per analizzare i dati di risposta ad un impulso di rotazione del robot
Ogni file cambia p ma tiene uguali i valori di Kp, Ki e Kd
In generale cambiera' Kp e p
"""

print("\n--- CERCO IL FILE fermo.txt ---")
try:
    print("--- CALCOLO L'ERRORE DEL GIROSCOPIO ---")
    t, r, l, e, o = np.loadtxt("datinuovabatteria/fermo.txt", unpack=True)

    errl = np.std(l)
    errt = np.std(np.diff(t))
    print("Errore sulla lettura errl = ", errl)
    print("Errore sulla lettura errt = ", errt)
    
except FileNotFoundError:
    print("\n!!! [ERRORE] non hai messo il file fermo.txt !!!")
    errl = float(input("Inserisci un valore di default per errl: "))
    errt = float(input("Inserisci un valore di default per errt: "))

print("\n--- SCELTA DEI PARAMETRI ---")
#p = 0.4
#Kp = 2.5 #float(input("Inserisci il valore di Kp: "))
Ki = 0.0
Kd = 0.0

pattern_ricerca = f"*_*_{Ki}_{Kd}_*.txt"
#pattern_ricerca = f"{p}_*_{Ki}_{Kd}_PID.txt"
#pattern_ricerca = f"*_{Kp}_{Ki}_{Kd}_PID.txt"
cartella = Path('valoriKubuoni')
#cartella = Path('.')
#cartella = Path('datinuovabatteria')
lista_file = [file.name for file in cartella.glob(pattern_ricerca)]

#print(lista_file)

dati_grafici = {}
ps_disordinati = set()
Kps_disordinati = set()

for nome_file in lista_file:
    print(f"\n--- CERCO DI APRIRE {nome_file} ---")
    try:
        print(f"--- APERTO IL FILE {nome_file} ---")
        
        p = float(nome_file.split('_')[0])
        Kp = float(nome_file.split('_')[1])
        
        t, r, l, e, o = np.loadtxt(cartella/nome_file, unpack=True)
        #tempo, riferimento, lettura, errore, output
        
        dati_grafici[(Kp, p)] = (t, r, l, e, o)
        ps_disordinati.add(p)
        Kps_disordinati.add(Kp)
        
    except FileNotFoundError:
        print(f"\n[ERRORE] Il file '{nome_file}' non esiste.")

ps = sorted(list(ps_disordinati))
Kps = sorted(list(Kps_disordinati))

col = len(Kps)
rig = len(ps)

print(f"\n--- CREAZIONE MATRICE DI GRAFICI ({rig} righe x {col} colonne) ---")
fig, axes = plt.subplots(rig, col, figsize=(5 * col, 9), sharex=True, sharey=True)

if rig == 1 and col == 1:
    axes = np.array([[axes]])
elif rig == 1:
    axes = axes[np.newaxis, :]
elif col == 1:
    axes = axes[:, np.newaxis]
    
print("\n--- COSTRUZIONE DELLA GRIGLIA ---")
for r_i, p in enumerate(ps):
    for c_i, Kp in enumerate(Kps):
        ax = axes[r_i, c_i]
        
        # Controllo se esiste il file combinato per questo specifico Kp e p
        if (Kp, p) in dati_grafici:
            t, r, l, e, o = dati_grafici[(Kp, p)]
            
            # Plot dei dati sul singolo sotto-grafico (ax)
            #ax.plot(t, r, label="riferimento", color='blue', linestyle='', marker='.')
            ax.errorbar(t, l, xerr=errt, yerr=errl, label="lettura", color='green', linestyle='-', marker=',')
            #ax.plot(t, e, label="errore", color='cyan', linestyle='', marker='.')
            #ax.plot(t, o, label="output", color='magenta', linestyle='', marker='.')
                
            ax.grid(True, linestyle=':', alpha=0.6)
            
        #else:
            # Se manca la combinazione (Kp, p), lascio il grafico vuoto scrivendoci sopra
            #ax.text(0.5, 0.5, 'Dato assente', ha='center', va='center', color='gray')
            
        # Titoli di riga e colonna solo sui bordi esterni per pulizia visiva
        if r_i == 0:
            ax.set_title(f"{Kp}", fontsize=12, fontweight='bold')
        if c_i == 0:
            ax.set_ylabel(f"{p}", fontsize=12, fontweight='bold', rotation=0, ha='right', va='center')
            
        # Etichette degli assi solo sull'ultima riga e prima colonna (grazie a sharex/sharey)
        if r_i == rig - 1:
            ax.set_xlabel("Tempo (s)")

# Aggiungo una legenda unica globale per non ripeterla in ogni grafico
handles, labels = axes[0, 0].get_legend_handles_labels()
if handles:
    fig.legend(handles, labels, loc='upper right', bbox_to_anchor=(0.99, 0.95))
    
plt.suptitle("Matrice risposte allo stimolo con potenza (righe) e Ku (colonne)", fontsize=16, y=0.98)
#plt.tight_layout(rect=[0, 0.5, 1, 0.5]) # Ottimizza gli spazi tra i grafici evitando sovrapposizioni
plt.show()
