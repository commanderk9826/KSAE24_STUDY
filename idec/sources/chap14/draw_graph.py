import matplotlib.pyplot as plt
import numpy as np

X = np.arange(0, 10)

Y1 = np.ones(10)		# ones()는 0으로 이루어진 넘파이 배열 생성
Y2 = X 
Y3 = X**2 

# 3개의 그래프를 하나의 축에 그린다. 
plt.plot(X, Y1, X, Y2, X, Y3)
plt.show()