import numpy as np
import matplotlib.pyplot as plt
from iminuit import Minuit
from pathlib import Path

"""
Questo file serve per analizzare i dati di risposta ad un impulso di rotazione del robot
Ogni file cambia p ma tiene uguali i valori di Kp, Ki e Kd
In generale cambiera' Ku e p
"""

def f(x, pa):
    # p[0] = y0    Offset iniziale
    # p[1] = A     Ampiezza
    # p[2] = w     Pulsazione
    # p[3] = t0    Offset temporale
    # p[4] = t1    Tempo finale
    #regime = y0 + A*cos(w*(x-t0))
    regime = pa[0] + pa[1] * np.cos(pa[2] * (x - pa[3]))
    return np.where(x < pa[3], pa[0], np.where(x > pa[4], pa[0], regime))

def dfdx(x, pa):
    #regime = -A*sin(w*(x-t0))*w
    regime = -pa[1] * np.sin(pa[2] * (x - pa[3])) * pa[2]
    return np.where(x < pa[3], 0, regime)
    
def fcn(par):
    errore = errl**2 + (dfdx(t, par)*errt)**2
    residui_pesati = (l - f(t, par))**2 / errore
    return np.sum(residui_pesati)

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

pattern_ricerca = f"*_*_{Ki}_{Kd}_PID.txt"
#pattern_ricerca = f"{p}_*_{Ki}_{Kd}_PID.txt"
#pattern_ricerca = f"*_{Ku}_{Ki}_{Kd}_PID.txt"
#cartella = Path('datipkp')
#cartella = Path('.')
cartella = Path('valoriKubuoni')
lista_file = [file.name for file in cartella.glob(pattern_ricerca)]

#print(lista_file)

dati_grafici = {}
ps_disordinati = set()
Kus_disordinati = set()

for nome_file in lista_file:
    print(f"\n--- CERCO DI APRIRE {nome_file} ---")
    try:
        print(f"--- APERTO IL FILE {nome_file} ---")
        
        p = float(nome_file.split('_')[0])
        Ku = float(nome_file.split('_')[1])
        
        t, r, l, e, o = np.loadtxt(cartella/nome_file, unpack=True)
        #tempo, riferimento, lettura, errore, output
        
        dati_grafici[(Ku, p)] = (t, r, l, e, o)
        ps_disordinati.add(p)
        Kus_disordinati.add(Ku)
        
    except FileNotFoundError:
        print(f"\n[ERRORE] Il file '{nome_file}' non esiste.")

p = float(input("Inserisci il valore di p da visualizzare: "))

ku = [chiave[0] for chiave in dati_grafici.keys() if chiave[1] == p]

if not ku:
    print(f"\n!!! [ERRORE] Non trovo Ku di p = {p} !!!")
    quit()
else:
    Ku = ku[0]
    print(f"Ku: {Ku}")

t, r, l, e, o = dati_grafici[(Ku, p)]

# p[0] = y0    Offset iniziale
# p[1] = A     Ampiezza
# p[2] = w     Pulsazione             2*np.pi*numero di punte tra t1 e t0/(t1-t0)
# p[3] = t0    Offset temporale
# p[4] = t1    Tempo finale

if(p == 0.6):
    par_iniziali = np.array([0, 0.67, 8.00, 7.37, 23.9]) #0.6
elif(p == 0.5):
    par_iniziali = np.array([0, 0.69, 7.62, 8.16, 26.8]) #0.5
elif(p == 0.4):
    par_iniziali = np.array([0, 0.43, 8.63, 7.14, 23.2]) #0.4
elif(p == 0.3):
    par_iniziali = np.array([0, 0.18, 11.1, 8.38, 24.1]) #0.3
else:
    quit()

m = Minuit(fcn, par_iniziali)
m.migrad()

y0, A, w, t0, t1 = m.values
curva_fit = f(t, m.values)

plt.figure(figsize=(8, 5))

plt.errorbar(t, l, yerr=errl, label="lettura", color='green', linestyle='', marker='.')
plt.plot(t, curva_fit, label="Fit Minuit", color='red', linestyle='-', linewidth=1)
                
print("\n--- PARAMETRI DEL FIT OTTENUTI ---")
print("Offset iniziale   = %f +- %f " % (y0, m.errors[0]))
print("Ampiezza          = %f +- %f " % (A, m.errors[1]))
print("Pulsazione        = %f +- %f " % (w, m.errors[2]))
print("Offset temporale  = %f +- %f " % (t0, m.errors[3]))
print("Tempo finale      = %f +- %f " % (t1, m.errors[4]))

"""
##################### --- BONTA' DEL FIT ---##############################
print("\n--- BONTA DEL FIT ---")
chi2 = m.fval  # Il valore minimo della FCN (somma dei residui quadratici)
ndf = len(t) - len(m.values)  # N_dati - N_parametri
chi2_ridotto = chi2 / ndf if ndf > 0 else float('nan')
            
print("Chi-quadro (chi2)       = %f" % chi2)
print("Gradi di liberta' (NDF) = %d" % ndf)
print("Chi-quadro ridotto      = %f" % chi2_ridotto)

# Una piccola nota interpretativa rapida (opzionale ma utile)
if 0.5 <= chi2_ridotto <= 1.5:
    print("Ottimo fit (vicino a 1).")
elif chi2_ridotto > 1.5:
    print("sottostima degli errori o modello incompleto.")
else:
    print("probabile sovrastima degli errori sperimentali.")
##########################################################################
"""

plt.title("Stimolo con Ku = %.2f e p = %.2f " % (Ku, p))
plt.xlabel("Tempo (s)")
plt.ylabel("Angolo (radianti)")
plt.legend()

Tu = 2*np.pi/w

Kp_new = 0.6*Ku
Ti = Tu/2
Ki_new = Kp_new/Ti
Td = Tu/8
Kd_new = Kp_new*Td

print(" Kp = ", Kp_new)
print(" Ki = ", Ki_new)
print(" Kd = ", Kd_new)

plt.show()
