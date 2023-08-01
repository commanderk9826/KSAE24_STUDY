numbers = [1, 2, 3, 4]  
slist = ['one', 'two', 'three', 'four']  
print(list(zip(numbers, slist)))

names = [ "KIM", "LEE", "PARK" ]
scores = [ 100, 99, 80 ] 

for n, s in zip(names, scores):
	print(n, s)  