single_tuple = ("apple",)    	# 쉼표가 끝에 있어야 한다. 
print(single_tuple)	
no_tuple = ("apple")    	# 쉼표가 없으면 튜플이 아니라 수식이 된다. 

fruits = ("apple", "banana", "grape")
print(fruits[1])		
# fruits[1] = "pear"		# 오류 발생!

fruits =("apple","banana","grape")
for f in fruits:
	print(f, end=" ")		# apple banana grape 출력 
myList = [1, 2, 3, 4] 
myTuple = tuple(myList)		# tuple()는 튜플을 생성하는 함수이다. 
print(myTuple)
