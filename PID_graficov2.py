import numpy as np
import matplotlib.pyplot as plt
from iminuit import Minuit

def f(x, p):
    # p[0] = y0    Offset iniziale
    # p[1] = A     Ampiezza
    # p[2] = w     Pulsazione
    # p[3] = phi   fase
    # p[4] = t0    Offset temporale
    
    #regime = y0 + A*cos(w*(x-t0) + fase)
    regime = p[0] + p[1] * np.cos(p[2] * (x - p[4]) + p[3])
    return np.where(x < p[4], p[0], regime)
    #return regime

def fcn(par):
    residui_pesati = (l - f(t, par)) / e_l
    return np.sum(residui_pesati**2)

print("\n--- SCELTA DEI PARAMETRI PID ---")
Kp  = 2.5 #0.5318442383620181 #float(input("Inserisci il valore di Kp: "))
Ki  = 0 #2.253450776719937 #float(input("Inserisci il valore di Ki: "))
Kd  = 0 #0.022039030844941594 #float(input("Inserisci il valore di Kd: "))
phi = 0 #float(input("Inserisci il valore di phi: "))

nome_file = "%.1f_%.1f_%.1f_%.1f_PID.txt" % (Kp, Ki, Kd, phi)
print(f"\nSto cercando di aprire il file: {nome_file}...")

try:
    print(f"--- APERTO IL FILE {nome_file} ---")
    t, r, l, e, o = np.loadtxt(nome_file, unpack=True)
    #tempo, riferimento, lettura, errore, output

    l_0 = l[:np.argmax(r != 0.0)]
    phi = phi*np.pi/180
    t0 = t[len(l_0)]
    #errl_0 = np.std(l_0, ddof=1)
    errl_0 = abs(np.mean(l_0))
    #if (errl_0 <= 0.0001):
    #    errl_0 = 0.0001
    print(errl_0)
    e_l = np.ones_like(l) * errl_0
        
    plt.figure(figsize=(8, 5))
    plt.plot(t, r, label="riferimento", color='blue', linestyle='-', marker=',')
    plt.errorbar(t, l, e_l, label="lettura", color='green', linestyle='', marker='.')
    #plt.plot(t, e, label="errore", color='cyan', linestyle='-', marker=',')
    #plt.plot(t, o, label="output", color='magenta', linestyle='-', marker=',')
    
    #"""
    #          PARAMETRI
    par_iniziali = np.array([0, 0.8, 7, 0, 7])
    
    # p[0] = y0    Offset iniziale
    # p[1] = A     Ampiezza
    # p[2] = w     Pulsazione
    # p[3] = fase   Fase
    
    m = Minuit(fcn, par_iniziali)
    m.print_level = 1
    m.migrad()
    
    y0, A, w, fase, t0 = m.values
    curva_fit = f(t, m.values)
    
    plt.plot(t, curva_fit, label="Fit Minuit", color='red', linestyle='-', linewidth=1)
    
    print("\n--- PARAMETRI DEL FIT OTTENUTI ---")
    print("Offset iniziale   = %f +- %f " % (y0, m.errors[0]))
    print("Ampiezza          = %f +- %f " % (A, m.errors[1]))
    print("Pulsazione        = %f +- %f " % (w, m.errors[2]))
    print("Fase              = %f +- %f " % (fase, m.errors[3]))
    print("Offset temporale  = %f +- %f " % (t0, m.errors[3]))
    """
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
    """
    print("\n--- CALCOLO COEFFICIENTI DEL PID ---")

    Ku = Kp
    Tu = 2*np.pi/w
   
    Kp_new = 0.6*Ku
    Ti = Tu/2
    Ki_new = Kp_new/Ti
    Td = Tu/8
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
