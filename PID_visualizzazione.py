import numpy as np
import matplotlib.pyplot as plt
from iminuit import Minuit

print("\n--- SCELTA DEL FILE DA VISUALIZZARE ---")

nome_file = input("Inserisci il nome del file .txt: ")

print(f"\nSto cercando di aprire il file: {nome_file}...")

try:
    print(f"--- APERTO IL FILE {nome_file} ---")
    t, r, l, e, o = np.loadtxt(nome_file, unpack=True)
    #tempo, riferimento, lettura, errore, output

    e_l = np.ones_like(l) * 0.01
        
    plt.figure(figsize=(8, 5))
    plt.plot(t, r, label="riferimento", color='blue', linestyle='-', marker=',')
    plt.errorbar(t, l, e_l, label="lettura", color='green', linestyle='', marker='.')
    plt.plot(t, e, label="errore", color='cyan', linestyle='-', marker=',')
    plt.plot(t, o, label="output", color='magenta', linestyle='-', marker=',')
    
    plt.title(nome_file)
    plt.xlabel("Tempo (s)")
    plt.ylabel("Angolo (radianti)")
    plt.legend()
    plt.show()
        
except FileNotFoundError:
    print(f"\n[ERRORE] Il file '{nome_file}' non esiste.")
