##
#	이 프로그램은 사용자로부터 이름과 나이를 받아서 다시 출력한다. 
#
def get_info():
	name = input("이름:")
	age = int(input("나이:"))
	return name, age		# 2개의 값을 반환한다. 
    
st_name, st_age = get_info()	# 반환된 값을 풀어서 변수에 저장한다.
print("이름은 ", st_name, "이고 나이는 ", st_age, "살입니다.")
