from tkinter import *
 
def process():
	print("버튼이 클릭되었습니다.")

window = Tk()
button = Button(window, text="클릭하세요!", command=process)
button.pack()


window.mainloop()