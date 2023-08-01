def  factorial(n):
    if n == 1 :
        return(1)
    else:
        return n * factorial(n-1)

n = eval(input("정수를 입력하시오:"))
print(n, "!= ", factorial(n))
