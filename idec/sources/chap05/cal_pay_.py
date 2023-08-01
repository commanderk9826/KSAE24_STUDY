##
#	이 프로그램은 주급을 계산한다. 
#

def weeklyPay( rate, hour ): 
    money = 0
    if (hour > 30): 
        money = rate*30 + 1.5*rate*(hour-30)
    else: 
        money = rate*hour
    return money

rate = int(input("시급을 입력하시오:"))
hour = int(input("근무 시간을  입력하시오:"))
print("주급은 " + str(weeklyPay(rate, hour)))
