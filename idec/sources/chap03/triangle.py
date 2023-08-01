a = int(input("삼각형의 한 변을 입력하시오: "))
b = int(input("삼각형의 한 변을 입력하시오: "))
c = int(input("삼각형의 한 변을 입력하시오: "))
if (a + b) > c and (b + c) > a and (a + c) > b :
	print("올바른 삼각형")
else :
	print("올바르지 않은 삼각형")