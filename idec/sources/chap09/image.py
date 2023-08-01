from tkinter import *
window = Tk()

canvas = Canvas(window,  width=500, height=300)
canvas.pack()

img = PhotoImage(file="d:\\starship.png")
canvas.create_image(20, 20, anchor=NW, image=img)

window.mainloop()
