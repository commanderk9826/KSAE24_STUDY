##
#	이 프로그램은 람다식을 이용하여 온도를 변환한다. 
#

f_temp = [0, 10, 20, 30, 40, 50]
c_temp = map(lambda x: (5.0/9.0)*(x-32.0), f_temp)
print(list(c_temp))