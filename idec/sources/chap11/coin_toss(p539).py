##
#	이 프로그램은 난수를 이용하여 동전 던지기 게임을 구현한다. 
#
import random
myList = [ "head", "tail" ]

while (True):
	response = input("동전 던지기를 계속하시겠습니까?( yes, no) ");
	if response == "yes":
		coin = random.choice(myList)
		print (coin)
	else :
		break