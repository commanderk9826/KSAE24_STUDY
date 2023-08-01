import functools
result = functools.reduce(lambda x,y: x+y, [1, 2, 3, 4])
print(result)