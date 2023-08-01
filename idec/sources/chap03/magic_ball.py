##
#	이 프로그램은 오늘의 운세를 출력한다. 
#
import random

print("행운의 매직볼로 오늘의 운세를 출력합니다. ")
answers = random.randint(1, 8)
if answers == 1:
    print("확실히 이루어집니다.")
elif answers == 2:
    print("좋아 보이네요")
elif answers == 3:
    print("믿으셔도 됩니다.")
elif answers == 4:
    print("저의 생각에는 no입니다.")
else: 
    print("다시 질문해주세요.")
