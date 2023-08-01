from tkinter import *

window = Tk()

Label(window , text="화씨").grid(row=0, column=0)
Label(window, text="섭씨").grid(row=1, column=0)

e1 = Entry(window).grid(row=0, column=1)
e2 = Entry(window).grid(row=1, column=1)

Button(window, text="화씨->섭씨").grid(row=2, column=1)
window.mainloop()
