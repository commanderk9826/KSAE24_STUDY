from tkinter import *

window = Tk()

b1 = Button(window, text="박스 #1", bg="red", fg="white")
b1.place(x=0, y=0)
b2 = Button(window, text="박스 #2", bg="green", fg="black")
b2.place(x=20, y=30)
b3 = Button(window, text="박스 #3", bg="orange", fg="white")
b3.place(x=40, y=60)

window.mainloop()
