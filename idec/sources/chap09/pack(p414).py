from tkinter import *
window = Tk()

Button(window, text="박스 #1", bg="red", fg="white").pack()
Button(window, text="박스 #2", bg="green", fg="black").pack()
Button(window, text="박스 #3", bg="orange", fg="white").pack()

window.mainloop()