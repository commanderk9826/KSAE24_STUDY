infile = open("input.txt", "r")
lines = infile.readlines()
for line in lines :
	print(line)
infile.close()