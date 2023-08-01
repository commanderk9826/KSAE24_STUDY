from tkinter import *				
window = Tk()					

button = Button(window,    text="클릭하세요!",
		bg="yellow",    fg="blue",	# 전경색과 배경색 설정
    	width=80,    height=2    	# 크기 설정
)

button.pack()					
window.mainloop()	