from tkinter import *

window = Tk()
choice = IntVar()

Label(window, text="가장 선호하는 프로그래밍 언어를 선택하시오",
      width=80).pack()

Radiobutton(window, text="Python", padx = 20,variable=choice, value=1).pack(anchor=W)
Radiobutton(window, text="C", padx = 20, variable=choice, value=2).pack(anchor=W)
Radiobutton(window, text="Java", padx = 20, variable=choice, value=3).pack(anchor=W)
Radiobutton(window, text="Swift", padx = 20, variable=choice, value=4).pack(anchor=W)

window.mainloop()
