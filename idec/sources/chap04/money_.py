TARGET = 2000			# 목표 금액
money = 1000			# 초기 자금
year = 0			# 연도
rate = 0.07			# 이자율

# 현재 금액이 목표 금액보다 작으면 반복한다. 
while money < TARGET :
	money = money + money * rate
	year = year + 1

print(year, "년")
