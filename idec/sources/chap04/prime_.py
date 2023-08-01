N_PRIMES = 50		# 찾아야 하는 소수의 개수
number = 2		# 2부터 시작한다. 
count = 0

while count < N_PRIMES :
	divisor = 2	# 나누는 수는 2부터 시작하여 하나씩 증가한다. 
	prime = True

	while divisor < number :
		if number % divisor == 0:	# 나누어지면 소수가 아니다. 
			prime = False
			break;
		divisor += 1
	if prime:		# 소수이면 소수 개수를 증가하고 출력한다. 
		count += 1
		print(number, end=" ")

	number += 1	# 다음 수로 간다. 
