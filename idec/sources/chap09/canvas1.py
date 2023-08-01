from tkinter import *
 
 
window = Tk()
w = Canvas(window, width=300, height=200)
w.pack()
w.create_rectangle(50, 25, 200, 100, fill="blue")
window.mainloop()
