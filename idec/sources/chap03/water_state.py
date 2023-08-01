##
#	이 프로그램은 온도에 따른 물의 상태를 출력한다. 
#
temp = float(input("온도를 입력하시오: "))

if temp <= 0 :
	print("물의 상태는 얼음입니다.")
elif temp > 0 and temp < 100 :		# 논리 연산자를 사용한다. 
	print("물의 상태는 액체입니다.")
else :
	print("물의 상태는 기체입니다.")