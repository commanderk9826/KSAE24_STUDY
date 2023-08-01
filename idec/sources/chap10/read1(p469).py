infile = open("input.txt", "r")
ch = infile.read(1)
while ch != "" :
	print(ch)
	ch = infile.read(1)
infile.close()