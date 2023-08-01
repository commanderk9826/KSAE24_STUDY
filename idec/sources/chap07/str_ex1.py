s1=input("첫 번째 문자열:")
s2=input("두 번째 문자열:")

list1 = list( set(s1) & set(s2) )		# 세트로 만들고 교집합 연산을 한다. 

print("\n공통적인 글자:", end=" ")
for i in list1:
    print(i, end=" ")