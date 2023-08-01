##
#	이 프로그램은 온도를 변환한다. 
#
def menu() :
    print("1. 섭씨 온도->화씨 온도")
    print("2. 화씨 온도->섭씨 온도")
    print("3. 종료")
    selection = int(input("메뉴를 선택하세요: "))
    return selection

def ctof(c) :
    temp = c*9.0/5.0 + 32
    return temp

def ftoc(f) :
    temp = (f-32.0)*5.0/9.0
    return temp

def input_f() :
    f = float(input("화씨 온도를 입력하시오: "))
    return f

def input_c() :
    c = float(input("섭씨 온도를 입력하시오: "))
    return c

def main() :
    while True:
        index = menu()
        if index == 1 :
            t = input_c()
            t2 = ctof(t)
            print("화씨 온도 = ", t2, "\n")
        elif index == 2 :
            t = input_f()
            t2 = ftoc(t)
            print("섭씨 온도 = ", t2, "\n")
        else :
            break
main()
