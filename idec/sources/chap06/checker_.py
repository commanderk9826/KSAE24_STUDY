table = [ ]

# 2차원 리스트를 화면에 출력한다.
def printList(mylist):
    for row in range(len(mylist)): 
        for col in range(len(mylist[0])): 
            print(mylist[row][col], end=" ")
        print()

# 2차원 리스트를 체커보드 형태로 초기화한다. 
def init(mylist):
    for row in range(len(mylist)): 
        for col in range(len(mylist[0])): 
            if (row+col)%2 == 0:
                table[row][col] = 1

# 2차원 리스트를 생성한다. 
for row in range(10): 
    table += [[0]*10]

init(table)
printList(table)
