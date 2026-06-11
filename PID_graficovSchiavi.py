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
    residui_pesati = (l_w - f(t, par)) / errl
    return np.sum(residui_pesati**2)

#print("\n--- SCELTA DEI PARAMETRI PID ---")
Kp  = 0 #0.5318442383620181 #float(input("Inserisci il valore di Kp: "))
Ki  = 0 #2.253450776719937 #float(input("Inserisci il valore di Ki: "))
Kd  = 0 #0.022039030844941594 #float(input("Inserisci il valore di Kd: "))
timer = 15 #float(input("Inserisci il valore di phi: "))

#nome_file = "%.1f_%.1f_%.1f_PID.txt" % (Kp, Ki, Kd)
#cartella = "./"
cartella = "datinuovabatteria/"
nome_file = "%.1f_%.1f_%.1f_%.1f_PID.txt" % (Kp, Ki, Kd, timer)
#nome_file = '0.4_0.3733021196832707_1.6004655198721427_0.013102590620354728_PIDfittato(w).txt'
nome_file = cartella+nome_file
print(f"\nSto cercando di aprire il file: {nome_file}...")

try:
    print(f"--- APERTO IL FILE {nome_file} ---")
    t, r_w, l_w, e_w, r_a, l_a, e_a, o = np.loadtxt(nome_file, unpack=True)
    #tempo, riferimento, lettura, errore, output

    #derivata = np.gradient(l, t)
    
    tempo_0 = t[:np.where(o != 0)[0][0]]
    t_0 = tempo_0[0]
    #print(t_0)
    lettura_0 = l_w[:np.where(o != 0)[0][0]]
    errl = np.mean(lettura_0)
    
    tempo_1 = t[np.where(o != 0)[0][0]:np.where(o != 0)[0][-1]]
    t_1 = tempo_1[0]
    #print(t_1)
    lettura_1 = l_w[np.where(o != 0)[0][0]:np.where(o != 0)[0][-1]]
    #derivata_centrale = derivata[len(tempo_0):len(tempo_0) + len(tempo_1)]
    #x_1 =  np.ones_like(tempo_1)
    
    tempo_2 = t[np.where(o != 0)[0][-1]:]
    t_2 = 12 #tempo_2[0]
    #print(t_2)
    lettura_2 = l_w[np.where(o != 0)[0][-1]:]
    #x_2 =  np.zeros_like(tempo_2)
    
    plt.figure(num="Grafico Velocita angolare", figsize=(8, 5))
    plt.errorbar(t, l_w, errl, label="lettura", color='blue', linestyle='-', marker=',')
    plt.plot(t, o, label="potenza", color='magenta', linestyle='-', marker=',')

    #"""    
    #          PARAMETRI
    par_iniziali = np.array([2.54, 20.9, 1.24, 0.098])
    
    # p[0] = y0    Offset iniziale
    # p[1] = C0    Valore gradino ottenuto
    # p[2] = T     Tempo del sistema
    # p[3] = t0    Tempo di ritardo
    
    m = Minuit(fcn, par_iniziali)

    #m.limits["x0"] = (0, 1)
    m.limits["x3"] = (0, 1)
    
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
    
    print("\n--- CALCOLO COEFFICIENTI DEL PID ---")
    M_0 = 0.4
    R, N = t0/T, (C0-y0)/T
    
    Kp_new = (4/3 + R/4)*M_0/(N*t0)
    
    Ti = t0*(32 + 6*R)/(13 + 8*R)
    Ki_new = Kp_new/Ti

    Td = t0*4/(11+2*R)
    Kd_new = Kp_new*Td
    
#    print(" Kp = %.2f \n Ki = %.2f \n Kd = %.2f" % (Kp_new, Ki_new, Kd_new))
    print("Kp =", Kp_new, ";")
    print("Ki =", Ki_new, ";")
    print("Kd =", Kd_new, ";")
#    """
    
    plt.title("IMPULSO")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Velocita' Angolare (radianti/s)")
    plt.legend()
    plt.show()
        
except FileNotFoundError:
    print(f"\n[ERRORE] Il file '{nome_file}' non esiste.")
