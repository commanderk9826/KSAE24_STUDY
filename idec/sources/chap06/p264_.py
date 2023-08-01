numbers =[10,3,7,1,9,4,2,8,5,6]
ascending_numbers =sorted(numbers)
print( ascending_numbers )

numbers =[10,20,30,40,50]

print("합=",sum(numbers))		# 항목의 합계를 계산한다. 
print("최대값=",max(numbers))		# 가장 큰 항목을 반환한다. 
print("최소값=",min(numbers))		# 가장 작은 항목을 반환한다. 

import random

numberList = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
print("랜덤하게 선택한 항목=", random.choice(numberList))

import random
movie_list = ["Citizen Kane", "Singin' in the Rain", "Modern Times",
              "Casablanca", "City Lights"]

item = random.choice(movie_list)
print ("랜덤하게 선택한 항목=", item)
