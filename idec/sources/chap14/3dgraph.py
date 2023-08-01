import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 

# 3차원 축(axis)을 얻는다. 
axis = plt.axes(projection='3d')
plt.show()

# 3차원 데이터를 넘파이 배열로 생성한다. 
Z = np.linspace(0, 1, 100)
X = Z * np.sin(30 * Z)
Y = Z * np.cos(30 * Z)

# 3차원 그래프를 그린다. 
axis.plot3D(X, Y, Z)