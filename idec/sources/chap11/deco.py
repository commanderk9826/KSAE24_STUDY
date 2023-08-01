def deco(text):
    return "######"+text + "######"

def greet(func, s):
    result = func(s)
    print(result)

greet(deco, "파이썬에 오신 것을 환영합니다!")