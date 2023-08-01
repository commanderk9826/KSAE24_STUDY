phrase  = input("문자열을 입력하시오: ")
  
acronym = ""

# 대문자로 만든 후에 단어들로 분리한다. 
for word in phrase.upper().split():
	acronym += word[0]		# 단어를 첫 글자만을 acronym에 추가한다. 
  
print( acronym )