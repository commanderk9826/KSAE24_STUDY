##
#	이 프로그램은 이분법을 구현한다. 
#

# 함수를 정의한다. 
def f(x):
    return(x**2-x-1)

def bisection_method(a, b, error):
    if f(a)*f(b) > 0:
        print("구간에서 근을 찾을 수 없습니다.")
    else:
        while (b - a)/2.0 > error:	# 오차를 계산한다. 
            midpoint = (a + b)/2.0	# 중점을 계산한다. 
            if f(midpoint) == 0:
                return(midpoint) 
            elif f(a)*f(midpoint) < 0: 
                b = midpoint
            else:
                a = midpoint
        
        return(midpoint)

answer = bisection_method(1, 2, 0.0001)

print("x**2-x-1의 근:", answer)
