import matplotlib.pyplot as plt
import numpy as np

# -2π에서 +2π까지 100개의 데이터를 균일하게 생성한다. 
X = np.linspace(-2 * np.pi, 2 * np.pi, 100)

# 넘파이 배열에 sin() 함수를 적용한다. 
Y1 = np.sin(X)
Y2 = 3 * np.sin(X)

plt.plot(X, Y1, X, Y2)
plt.show()