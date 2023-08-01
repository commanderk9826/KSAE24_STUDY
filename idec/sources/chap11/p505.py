i = -20  
print(abs(i))
 
f = -30.92  
print(abs(f))


mylist = [1, 3, 4, 6]  		# 모든 값이 참이다. 
print(all(mylist))
mylist = [1, 3, 4, 0]  		# 하나의 값이 거짓이다.
print(all(mylist))

mylist = [True, 0, False, 0]  	# 하나의 값만 참이다. 
print(all(mylist))

mylist = [0, 1, 2, 3]                              
print(any(mylist))			

mylist = [0, False]  
print(any(mylist))
