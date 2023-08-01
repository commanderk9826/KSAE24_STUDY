sentence = input("문자열을 입력하시오: ")

table ={"alphas":0,"digits":0,"spaces":0 }

for i in sentence:
   if i.isalpha():
      table["alphas"]+=1
   if i.isdigit():
      table["digits"]+=1
   if i.isspace():
      table["spaces"]+=1

print(table)
