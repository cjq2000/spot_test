import numpy as np
import matplotlib.pyplot as plt

alpha = 1
t = np.linspace(0, 2*np.pi, num=1000)

x = alpha * np.sqrt(2) * np.cos(t) / (np.sin(t)**2 + 1)
y = alpha * np.sqrt(2) * np.cos(t) * np.sin(t) / (np.sin(t)**2 + 1)

plt.plot(x, y)
plt.show()