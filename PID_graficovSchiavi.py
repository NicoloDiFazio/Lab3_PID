import numpy as np
import matplotlib.pyplot as plt
from iminuit import Minuit

#def f(x, p):
#    return p[0]*x+p[1]

def f(x, p):
    # p[0] = y0    Offset iniziale
    # p[1] = C0    Valore gradino
    # p[2] = T     Tempo del sistema
    # p[3] = t0    Tempo di ritardo
    #y0, C0, T, t0 = p
    #salita = y0 + (C0 - y0)*(1-np.exp(-(x-(t_0+t0))/T))
    inizio = 0
    salita = p[0] + (p[1] - p[0])*(1-np.exp(-(x-(t_1 + p[3]))/p[2]))
    discesa = 0
    return np.where(x < t_1 + p[3], inizio, np.where(x > t_2, discesa, salita))
    
def fcn(par):
    residui_pesati = (l - f(t, par)) / errl
    return np.sum(residui_pesati**2)

print("\n--- SCELTA DEI PARAMETRI PID ---")
Kp  = 0 #0.5318442383620181 #float(input("Inserisci il valore di Kp: "))
Ki  = 0 #2.253450776719937 #float(input("Inserisci il valore di Ki: "))
Kd  = 0 #0.022039030844941594 #float(input("Inserisci il valore di Kd: "))
timer = 20 #float(input("Inserisci il valore di phi: "))

#nome_file = "%.1f_%.1f_%.1f_PID.txt" % (Kp, Ki, Kd)
nome_file = "%.1f_%.1f_%.1f_%.1f_PID.txt" % (Kp, Ki, Kd, timer)
print(f"\nSto cercando di aprire il file: {nome_file}...")

try:
    print(f"--- APERTO IL FILE {nome_file} ---")
    t, r, l, e, o = np.loadtxt(nome_file, unpack=True)
    #tempo, riferimento, lettura, errore, output

    #derivata = np.gradient(l, t)
    
    tempo_0 = t[:np.where(o != 0)[0][0]]
    t_0 = tempo_0[0]
    print(t_0)
    lettura_0 = l[:np.where(o != 0)[0][0]]
    errl = 0.01 #np.mean(lettura_0)
    #x_0 =  np.zeros_like(tempo_0)
    
    tempo_1 = t[np.where(o != 0)[0][0]:np.where(o != 0)[0][-1]]
    t_1 = tempo_1[0]
    print(t_1)
    lettura_1 = l[np.where(o != 0)[0][0]:np.where(o != 0)[0][-1]]
    #derivata_centrale = derivata[len(tempo_0):len(tempo_0) + len(tempo_1)]
    #x_1 =  np.ones_like(tempo_1)
    e_l = np.ones_like(tempo_1) * errl
    
    tempo_2 = t[np.where(o != 0)[0][-1]:]
    t_2 = tempo_2[0]
    print(t_2)
    lettura_2 = l[np.where(o != 0)[0][-1]:]
    #x_2 =  np.zeros_like(tempo_2)
    
    plt.figure(figsize=(8, 5))
    #plt.plot(tempo_0, lettura_0, label="prima parte", color='blue', linestyle='', marker='.') 
    #plt.plot(tempo_1, lettura_1, label="seconda parte", color='blue', linestyle='', marker='.')
    #plt.plot(tempo_2, lettura_2, label="terza parte", color='blue', linestyle='', marker='.')
    plt.errorbar(t, l, errl, label="lettura", color='blue', linestyle='', marker='.')
    plt.plot(t, r, label="potenza", color='magenta', linestyle='-', marker=',')

    #          PARAMETRI
    par_iniziali = np.array([0, 30, 1.7, .1])
    
    # p[0] = y0    Offset iniziale
    # p[1] = C0    Valore gradino ottenuto
    # p[2] = T     Tempo del sistema
    # p[3] = t0    Tempo di ritardo
    
    m = Minuit(fcn, par_iniziali)
    m.print_level = 1
    m.migrad()
    
    y0, C0, T, t0 = m.values
    curva_fit = f(t, m.values)
    
    plt.plot(t, curva_fit, label="Fit Minuit", color='red', linestyle='-', linewidth=1)
    
    print("\n--- PARAMETRI DEL FIT OTTENUTI ---")
    print("Offset iniziale         = %f +- %f " % (y0, m.errors[0]))
    print("Altezza del gradino     = %f +- %f " % (C0, m.errors[1]))
    print("Tempo del sistema       = %f +- %f " % (T, m.errors[2]))
    print("Tempo di ritardo        = %f +- %f " % (t0, m.errors[3]))
    
    # --- BONTA' DEL FIT ---
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
    # -------------------------------------
    
    print("\n--- CALCOLO COEFFICIENTI DEL PID ---")
    M_0 = r[len(tempo_0)]
    R, N = t0/T, (C0-y0)/T
    
    Kp_new = (4/3 + R/4)*M_0/(N*t0)
    
    Ti = t0*(32 + 6*R)/(13 + 8*R)
    Ki_new = Kp_new/Ti

    Td = t0*4/(11+2*R)
    Kd_new = Kp_new*Td
    
#    print(" Kp = %.2f \n Ki = %.2f \n Kd = %.2f" % (Kp_new, Ki_new, Kd_new))
    print(" Kp = ", Kp_new)
    print(" Ki = ", Ki_new)
    print(" Kd = ", Kd_new)
#    """
    
    plt.title("IMPULSO")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Velocita' Angolare (radianti/s)")
    plt.legend()
    plt.show()
        
except FileNotFoundError:
    print(f"\n[ERRORE] Il file '{nome_file}' non esiste.")
