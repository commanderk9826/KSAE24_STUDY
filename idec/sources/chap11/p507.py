print(sum([1, 2, 3 ])  )
print(sum([1, 2, 3], 20)  )

print(len("All's well that ends well. "))
print(len([1, 2, 3, 4, 5]))
s  = 'abcdefg'       
print(list(s)  )
  
t = (1, 2, 3, 4, 5, 6)  
print(list(t)  )

def square(n):  
  return n*n  
  
mylist = [1, 2, 3, 4, 5]  
result = list(map(square, mylist))
print(result)  
