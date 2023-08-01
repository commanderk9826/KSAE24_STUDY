##
#	이 프로그램은 라스베가스에서 목표 금액을 획득할 확률을 계산한다. 
#
import random

initial_money = 50
goal = 250
wins = 0

for i in range(100) :		# 라스베가스에 100번 간다.
    cash = initial_money
    while cash > 0 and cash < goal :	# 돈이 0이거나 250불을 따면 반복 중단
        number = random.randint(1, 2)
        if number == 1 : 
            cash = cash + 1		# $1을 딴다.
        else :
            cash = cash - 1		# $1을 잃는다.
    if cash == goal : wins = wins + 1     

print("초기 금액 $%d" % initial_money)
print("목표 금액 $%d" % goal)
print("100번 중에서 %d번 성공" % wins)
