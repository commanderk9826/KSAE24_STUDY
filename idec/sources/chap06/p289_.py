s = [ 	[ 1, 2, 3, 4, 5 ] ,
	[ 6, 7, 8, 9, 10 ], 
	[11, 12, 13, 14, 15 ] ]

# 행과 열의 개수를 구한다. 
rows = len(s)
cols = len(s[0])

for r in range(rows):
	for c in range(cols):
		print(s[r][c], end=",")
	print()
