infile = open("proverbs.txt", "r")

for line in infile:
	line = line.rstrip()		# 오른쪽 공백 문자를 없앤다. 
	word_list = line.split()	# 단어들로 분리한다. 
	for word in word_list:	# 리스트에 들어 있는 단어들을 출력한다. 
		print(word);
infile.close() 