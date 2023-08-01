values = [ 1, 2, 3, 4, 5]
print(max(values))
print(min(values))

seasons = ['Spring', 'Summer', 'Fall', 'Winter']
print(list(enumerate(seasons)))
print(list(enumerate(seasons, start=1)))


def myfilter(x):  
	return x > 3  

result = filter(myfilter, (1, 2, 3, 4, 5, 6))  
print(list(result)) 
