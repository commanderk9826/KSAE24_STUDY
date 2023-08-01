##
#	이 프로그램은 사용자의 입력을 검증한다. 
#
print("========================")
print("메뉴 1번: 치즈 버거")
print("메뉴 2번: 치킨 버거")
print("메뉴 3번: 불고기 버거")
print("========================")

selection = int(input("메뉴를 선택하세요:"))

if selection >= 1 and selection <= 3 :
	print("메뉴 ", selection)
else :
	print("잘못 입력하셨습니다.")