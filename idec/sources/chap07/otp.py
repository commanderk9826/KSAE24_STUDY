import random
  
s = "0123456789"		# 대상 문자열
passlen = 4			# 패스워드 길이


p =  "".join(random.sample(s, passlen ))
print(p)