import numpy as np

sigma=np.array([
    [19 / 16, -15 * np.sqrt(3) / 16,],
    [-15 * np.sqrt(3) / 16, 49 / 16]
])

eVa,eVe=np.linalg.eig(sigma)
print("eigen value:\n", eVa)
print("eigen vector:\n", eVe)