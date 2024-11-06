import numpy as np
import matplotlib.pyplot as plt

lefts = np.load('lefts.npy')
rights = np.load('rights.npy')
errors = np.load('errors.npy')

plt.plot(errors, lefts, 'r', label='Left')
plt.plot(errors, rights, 'b', label='Right')
plt.legend()
plt.show()