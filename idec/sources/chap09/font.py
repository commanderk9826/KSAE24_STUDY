from tkinter import *

window = Tk()
canvas = Canvas(window, width=600, height=200, bg = '#afeeee')
canvas.create_text(200, 100, fill="darkblue", font="Times 30 italic bold",
                        text="This is a text example.")
canvas.pack()
window.mainloop()