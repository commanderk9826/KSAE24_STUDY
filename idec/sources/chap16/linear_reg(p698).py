import matplotlib.pylab as plt
from sklearn import linear_model

reg = linear_model.LinearRegression()		# 선형 회귀 객체 생성
X = [[174], [152], [138], [128], [186]]		# 학습 예제
y = [71, 55, 46, 38, 88]				# 정답

reg.fit(X, y)						# 학습 함수
print(reg.coef_)		# 직선의 기울기
print(reg.intercept_) 	# 직선의 절편 
print(reg.score(X, y))	# 학습 점수
print(reg.predict([[178]]))

# 학습 데이터를 산포도로 그린다. 
plt.scatter(X, y, color='black')

# 학습 데이터를 입력으로 하여 예측값을 계산한다. 직선을 가지고 예측하기 때문에 직선 상의 점이 된다. 
y_pred = reg.predict(X)

# 예측값으로 선그래프를 그린다. 
# 직선이 그려진다. 
plt.plot(X, y_pred, color='blue', linewidth=3)		
plt.show()