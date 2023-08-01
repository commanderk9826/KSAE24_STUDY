# 동적으로 2차원 리스트를 생성한다. 
rows = 3
cols = 5

s = [ ]
for row in range(rows): 
	s += [[0]*cols]		# 2차원 리스트끼리 합쳐진다. 

print("s =", s)


rows = 3
cols = 5

s = [ ([0] * cols) for row in range(rows) ]

print("s =", s)
