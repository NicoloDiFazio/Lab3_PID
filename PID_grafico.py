import numpy as np
import matplotlib.pyplot as plt
from iminuit import Minuit

def f(x, p):
    # p[0] = y0    Offset iniziale
    # p[1] = C0    Valore gradino
    # p[2] = T     Tempo del sistema
    # p[3] = t0    Tempo di ritardo
    #y0, C0, T, t0 = p
    #salita = y0 + (C0 - y0)*(1-np.exp(-(x-(t_0+t0))/T))
    salita = p[0] + (p[1] - p[0])*(1-np.exp(-(x-(t_0+p[3]))/p[2]))
    
    return np.where(x < t_0 + p[3], p[0], salita)

def fcn(par):
    residui_pesati = (l - f(t, par)) / e_l
    return np.sum(residui_pesati**2)

print("\n--- SCELTA DEI PARAMETRI PID ---")
Kp  = 2 #0.5318442383620181 #float(input("Inserisci il valore di Kp: "))
Ki  = 0 #2.253450776719937 #float(input("Inserisci il valore di Ki: "))
Kd  = 0 #0.022039030844941594 #float(input("Inserisci il valore di Kd: "))
phi = 10 #float(input("Inserisci il valore di phi: "))

nome_file = "%.1f_%.1f_%.1f_PID.txt" % (Kp, Ki, Kd)
print(f"\nSto cercando di aprire il file: {nome_file}...")

try:
    print(f"--- APERTO IL FILE {nome_file} ---")
    t, r, l, e, o = np.loadtxt(nome_file, unpack=True)
    #tempo, riferimento, lettura, errore, output

    l_0 = l[r == 0.0]
    phi = phi*np.pi/180
    #t_0 = t[len(l_0)]
    #errl_0 = np.std(l_0, ddof=1)
    #if (errl_0 <= 0.001):
    errl_0 = 0.001
    e_l = np.ones_like(l) * errl_0
    print(e_l[0])
    
    plt.figure(figsize=(8, 5))
    plt.plot(t, r, label="riferimento", color='blue', linestyle='-', marker=',')
    plt.errorbar(t, l, e_l, label="lettura", color='green', linestyle='', marker='.')
    plt.plot(t, e, label="errore", color='cyan', linestyle='-', marker=',')
    plt.plot(t, o, label="output", color='magenta', linestyle='-', marker=',')
    """

    #          PARAMETRI
    par_iniziali = np.array([np.mean(l_0), 0.32, 0.09, 0.14])
    
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
    
    R, M_0, N = t0/T, r[len(l_0)], (C0-y0)/T
    Kp_new = (4/3 + R/4)*M_0/(N*t0)
    Ti = t0*(32 + 6*R)/(13 + 8*R)
    Ki_new = Kp_new/Ti
    Td = t0*(4)/(11 + 2*R)
    Kd_new = Kp_new*Td
    
#    print(" Kp = %.2f \n Ki = %.2f \n Kd = %.2f" % (Kp_new, Ki_new, Kd_new))
    print(" Kp = ", Kp_new)
    print(" Ki = ", Ki_new)
    print(" Kd = ", Kd_new)
#    """
    plt.title("PID con Kp=%.2f, Ki=%.2f, Kd=%.2f e phi=%.2f" % (Kp, Ki, Kd, phi))
    plt.xlabel("Tempo (s)")
    plt.ylabel("Angolo (radianti)")
    plt.legend()
    plt.show()
        
except FileNotFoundError:
    print(f"\n[ERRORE] Il file '{nome_file}' non esiste.")
