import numpy as np
import matplotlib.pyplot as plt

pure = np.linspace(1, 10, 100)		# 1부터 10까지 100개의 데이터 생성
noise = np.random.normal(0, 1, 100)	# 평균이 0이고 표준편차가 1인 100개의 난수 생성

# 넘파이 배열 간 덧셈 연산, 요소별로 덧셈이 수행된다. 
signal = pure + noise			

# 선 그래프를 그린다. 
plt.plot(signal)
plt.show()