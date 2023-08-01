##
#	이 프로그램은 자판기에서 거스름돈을 계산한다.  
#
itemPrice = int(input("물건값을 입력하시오: "))
note = int(input("1000원 지폐개수: "))
coin500 = int(input("500원 동전개수: "))
coin100 = int(input("100원 동전개수: "))

change = note*1000 + coin500*500 + coin100*100 - itemPrice

# 거스름돈(500원 동전 개수)을 계산한다. 
nCoin500 = change//500
change = change%500

# 거스름돈(100원 동전 개수)을 계산한다. 
nCoin100 = change//100
change = change%100

# 거스름돈(10원 동전 개수)을 계산한다. 
nCoin10 = change//10
change = change%10

# 거스름돈(1원 동전 개수)을 계산한다. 
nCoin1 = change

print("500원=", nCoin500, "100원=", nCoin100, "10원=", nCoin10, "1원=", nCoin1) 
