import numpy as np
import matplotlib.pyplot as plt
from iminuit import Minuit

p = np.array([0.3, 0.4, 0.5, 0.6])
Ku = np.array([2.8, 2.25, 2.25, 1.4727])

plt.figure(figsize=(8, 5))
plt.plot(p, Ku, label="Ku(p)", color='blue', linestyle='-', marker='.')

plt.title("Ku in funzione di p")
plt.xlabel("Potenza")
plt.ylabel("Ku iniziale (da cui ricavare Kp, Ki, Kd)")
plt.legend()

plt.show()
