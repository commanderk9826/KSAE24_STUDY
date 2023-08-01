from tkinter import *
from random import *

NUM_MOLES = 3						# 두더지 개수

window = Tk()						# 루트 윈도우 생성
window.title("두더지 게임")

moleFrame = Frame(window)			# 첫 번째 프레임 컨테이너 생성
moleFrame.grid(row=0, column=0)	# 첫 번째 프레임을 루트 윈도우에 배치

statusFrame = Frame(window)		# 두 번째 프레임 컨테이너 생성
statusFrame.grid(row=0, column=1)	# 두 번째 프레임을 루트 윈도우에 배치
hitsLabel = Label(statusFrame, text="0 Hits")
hitsLabel.pack()
missedLabel = Label(statusFrame, text="0 Misses")
missedLabel.pack()

mole_image = PhotoImage(file="mole.png")		# 이미지를 읽는다. 
no_mole_image = PhotoImage(file="no_mole.png")

numHits=0							# 획득 점수
numMissed=0						# 실패 횟수

def mole_hit(c):					# 사용자가 두더지를 잡았는지를 체크한다. 
    global numHits, numMissed, molesList, missedLabel, hitsLabel
    if molesList[c]["text"] == "mole":
        numHits += 1
        hitsLabel["text"] = str(numHits)+" Hits"
    else:
        numMissed += 1
        missedLabel["text"] = str(numMissed)+" Misses"

molesList = []						# 버튼들이 저장된다. 

def init():
    count=0
    for r in range(NUM_MOLES):		# 버튼을 만들어서 격자 형태로 배치한다. 
        for c in range(NUM_MOLES):
            button = Button(moleFrame, command=lambda c=count: mole_hit(c))
            button["image"] = no_mole_image
            button["text"] = "no mole"
            button.grid(row=r, column=c)
            molesList.append(button)
            count += 1

def update():						# 랜덤하게 버튼에 두더지를 심는다. 
    global molesList
    for i in range(NUM_MOLES*NUM_MOLES):	# 전체 버튼을 초기화한다. 
        button = molesList[i]
        button["text"] = "no mole"
        button["image"] = no_mole_image
    x = randint(0, NUM_MOLES*NUM_MOLES-1)	# 난수를 발생한다. 
    molesList[x]["image"] = mole_image		# 두더지 영상으로 바꾼다.
    molesList[x]["text"] = "mole"
    window.after(3000, update)		# 3초 지나면 다시 호출되게 한다. 

init()
update()
window.mainloop()