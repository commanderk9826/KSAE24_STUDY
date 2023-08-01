from tkinter import *
 
window = Tk()
 
btn1 = Button(window, text="1 출력", command=lambda: print(1, "버튼이 클릭"))
btn1.pack(side=LEFT)
btn2 = Button(window, text="2 출력", command=lambda: print(2, "버튼이 클릭"))
btn2.pack(side=LEFT)
 
quitBtn = Button(window, text="QUIT", fg="red", command=quit)
quitBtn.pack(side=LEFT)
 
mainloop()