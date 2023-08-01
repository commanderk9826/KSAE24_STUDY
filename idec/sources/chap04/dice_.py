##
#	이 프로그램은 주사위 2개의 합이 6인 경우를 전부 출력한다. 
#
for a in range(1, 7) :
	for b in range(1, 7) :
			if a + b == 6 :
				print(f"첫 번째 주사위={a} 두 번째 주사위={b}" )
