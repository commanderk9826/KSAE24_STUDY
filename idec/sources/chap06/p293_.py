matrix = [[i for i in range(5)] for _ in range(6)]
print(matrix)

matrix = [ [0, 0, 0],     [1, 1, 1],     [2, 2, 2] ]
result = [num for row in matrix for num in row]
print(result)

matrix = [ [0, 0, 0],     [1, 1, 1],     [2, 2, 2] ]
result = []
for row in matrix:
     for num in row:
         result.append(num)
print(result)
