from tkinter import *

window = Tk()
Label(window, text="선호하는 언어를 모두 선택하시오:", width=80).grid(row=0)

value1 = IntVar()
Checkbutton(window, text="Python", variable=value1).grid(row=1, sticky=W)
value2 = IntVar()
Checkbutton(window, text="C", variable=value2).grid(row=2, sticky=W)
value3 = IntVar()
Checkbutton(window, text="Java", variable=value3).grid(row=3, sticky=W)
value4 = IntVar()
Checkbutton(window, text="Swift", variable=value4).grid(row=4, sticky=W)

window.mainloop()