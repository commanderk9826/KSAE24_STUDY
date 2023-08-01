list1 = [10, 20, 30, 40, 50]

list2 = [sum(list1[0:x+1]) for x in range(0, len(list1))]

print("원래 리스트: ",list1)
print("새로운 리스트: ",list2)
