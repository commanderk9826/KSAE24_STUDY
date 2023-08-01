def  fib( n):
    if( n==0 ):
        return 0
    elif( n==1 ):
        return 1
    else:
        return fib(n-1) + fib(n-2)

n = int(input("정수를 입력하시오:"))
print(n, "번째 피보나치 수는", fib(n))
