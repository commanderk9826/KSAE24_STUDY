##
#	이 프로그램은 파이값을 계산한다. 
#
divisor = 1.0
divident = 4.0
sum = 0.0
loop_count = int(input("반복횟수:"))
while(loop_count > 0) :
	sum = sum + divident / divisor
	divident = -1.0 * divident
	divisor = divisor + 2
	loop_count = loop_count - 1;
print("Pi = %f" % sum)
