def sum(numbers) :
    total = 0
    for i in range(len(numbers)) :
        for j in range(len(numbers[0])) :
             total = total + numbers[i][j]
    return total
