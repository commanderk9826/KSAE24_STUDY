import numpy as np
import matplotlib.pyplot as plt

m, sigma = 10, 2
Y1 = np.random.randn(10000)
Y2 = m+sigma*np.random.randn(10000)

plt.figure(figsize=(10,6))		# 그래프의 크기 설정
plt.hist(Y1, bins=20)
plt.hist(Y2, bins=20)
plt.show()