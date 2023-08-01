list_a = [ 1, 2, 3, 4, 5 ]
list_b = [ 10, 20, 30, 40, 50 ]
func = lambda x, y : x + y
result = map(func, list_a, list_b)
print(list(result))