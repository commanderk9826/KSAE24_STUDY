STUDENTS = 5 
lst = [] 
count=0

for i in range(STUDENTS):
    value = int(input("성적을 입력하시오: "))
    lst.append(value)

print("\n성적평균=", sum(lst) / len(lst))
print("최대점수=", max(lst))
print("최소점수=", min(lst))

for score in lst:
    if score >= 80:
        count += 1
print("80점 이상=", count)
