def get_sum(start, end):
	sum = 0
	for i in range(start, end+1):
		sum += i
	return sum

value = get_sum(1, 10)
