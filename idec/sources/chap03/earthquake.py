##
#	이 프로그램은 리히터 규모를 받아서 피해정도를 출력한다. 
#
scale = float(input("리히터 규모를 입력하시오: "))

if scale >= 8.0 :
	print("대부분의 구조물이 파괴됩니다. ")
elif scale >= 7.0 :
	print("지표면에 균열이 발생합니다.")
elif scale >= 4.0 :
	print("빈약한 건물에 큰 피해가 있습니다. ")
elif scale >= 2.0 :
	print("물건들이 흔들리거나 떨어집니다.")
else :
	print("지진계에 의해서만 탐지 가능합니다. ")
