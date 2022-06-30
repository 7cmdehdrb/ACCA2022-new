import numpy as np

H = np.array([[1, 2], [3, 4]])
b = np.array([[5, 6], [7, 8]])

print(H * b)
print(H @ b)
print(np.dot(H, b))
