from tkinter import *

window = Tk()
window.geometry("600x200") 

def callback(event):
    print(event.x, event.y, "에서 마우스 이벤트 발생")

window.bind("<Button-1>", callback)
window.mainloop()