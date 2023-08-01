print(sorted([4, 2, 3, 5, 1]))
myList = [4, 2, 3, 5, 1]
myList.sort()
print(myList)

print(sorted({3: 'D', 2: 'B', 5: 'B', 4: 'E', 1: 'A'}))


print(sorted("The health know not of their health, but only the sick".split(), key=str.lower))

students = [
        ('홍길동', 3.9, 20160303),
        ('김철수', 3.0, 20160302),
        ('최자영', 4.3, 20160301),
]
print(sorted(students, key=lambda student: student[2]))