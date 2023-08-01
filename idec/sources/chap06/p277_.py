lst = [1, 2, 3, 4, 5, 6, 7, 8]
lst[0:3] = ['white', 'blue', 'red']
print(lst)

lst = [1, 2, 3, 4, 5, 6, 7, 8]
lst[::2] = [99, 99, 99, 99]
print(lst)

lst = [1, 2, 3, 4, 5, 6, 7, 8]
lst[:] = [ ]
print(lst)
