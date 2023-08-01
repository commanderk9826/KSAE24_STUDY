from tkinter import *

def show():
   print("이름:", e1.get(), "\n나이", e2.get())
   
window = Tk()
Label(window, text="이름").grid(row=0)
Label(window, text="나이").grid(row=1)

e1 = Entry(window)
e2 = Entry(window)

e1.grid(row=0, column=1)
e2.grid(row=1, column=1)

Button(window, text='보이기', command=show).grid(row=3, column=1, sticky=W+S+E+N, pady=4)
Button(window, text='종료', command=window.quit).grid(row=3, column=0, sticky=W, pady=4)

window.mainloop( )