transposed = []
matrix = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]

print("원래 행렬=", matrix)
# 열의 개수만큼 반복한다. 
for i in range(len(matrix[0])):
    transposed_row = []
    for row in matrix:		# 행렬의 각 행에 대하여 반복
        transposed_row.append(row[i])	# i번째 요소를 row에 추가한다. 
    transposed.append(transposed_row)

print("전치 행렬=", transposed)
