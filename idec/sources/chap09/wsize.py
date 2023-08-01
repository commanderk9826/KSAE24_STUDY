from tkinter import *

window = Tk()
window.geometry("600x100") 		# Width x Height

Button(window, text="박스 #1", width=10, height=1).pack()
Button(window, text="박스 #2", width=10, height=1).pack()
Button(window, text="박스 #3", width=10, height=1).pack()

window.mainloop()