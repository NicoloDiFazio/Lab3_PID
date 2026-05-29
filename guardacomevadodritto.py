import numpy as np
import matplotlib.pyplot as plt
from iminuit import Minuit

cartella = "davedere/"
nomifiles = ["0.3_4.0_noPID.txt", "0.4_4.0_noPID.txt", "0.5_4.0_noPID.txt", "0.6_4.0_noPID.txt", "0.3_1.68_5.95936795516598_0.11840181799620855_PIDfittato.txt", "0.4_1.3499999999999999_3.7105538480072124_0.12279164207378301_PIDfittato.txt", "0.5_1.3499999999999999_3.2730709864795053_0.1392041302746285_PIDfittato.txt", "0.6_0.88362_2.2505991596539565_0.08673071580192564_PIDfittato.txt"]

p = [0.3, 0.4, 0.5, 0.6]
Ks = ["senza PID", "con PID", "con PID velocita' angolare"]
colori = ["blue", "green", "magenta"]

ultimo_file = "0.4_0.3733021196832707_1.6004655198721427_0.013102590620354728_PIDfittato(w).txt"

fig, axs = plt.subplots(3, 4, sharex=True, sharey=True)

for k, nome_file in enumerate(nomifiles):
    if(k < 4):
        i = k
        j = 0
    else:
        i = k - 4
        j = 1
    #print(k, i, j)
    #print(nome_file)
    
    t, r, l, e, o = np.loadtxt(cartella + nome_file, unpack=True)
    axs[j, i].plot(t, l, label=f"{p[i]} {Ks[j]}", color=f"{colori[j]}")
    if(j == 0): axs[j, i].set_title(f"{p[i]}")

    
    axs[j, i].grid(True, linestyle=':', alpha=0.6)

t, r_w, l_w, e_w, r_a, l_a, e_a, o = np.loadtxt(cartella + ultimo_file, unpack=True)
j = 2
i = 1
axs[j, i].plot(t, l_a, label=f"{p[i]} {Ks[j]}", color=f"{colori[j]}")
axs[j, i].grid(True, linestyle=':', alpha=0.6)

fig.legend(loc='lower right')
fig.tight_layout()
plt.show()
