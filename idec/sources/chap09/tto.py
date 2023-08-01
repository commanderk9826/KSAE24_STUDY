from tkinter import *

# i번째 버튼을 누를 수 있는지 검사한다. 누를 수 있으면 X나 O를 표시한다. 
def checked(i):
    global player
    button = list[i]	# 리스트에서 I번째 버튼 객체를 가져온다. 

	  # 버튼이 초기상태가 아니면 이미 누른 버튼이므로 아무것도 하지 않고 리턴한다.
    if   button["text"] != "            ":
        return
    button["text"] = "     " + player+"      "
    button["bg"] = "yellow"
    if   player=="X":
        player = "O"
        button["bg"] = "yellow"
    else :
        player = "X"
        button["bg"] = "lightgreen"


window = Tk()		# 윈도우를 생성한다.  
player="X"		# 시작은 플레이어 X이다. 
list = []

# 9개의 버튼을 생성하여 격자 형태로 윈도우에 배치한다. 
for i in range(9):
    b = Button(window, text="            ", command=lambda k=i: checked(k))
    b.grid(row=i//3, column=i%3)
    list.append(b)	# 버튼 객체를 리스트에 저장한다. 

window.mainloop()
