from tkinter import *				# tkinter 모듈을 포함

window = Tk()					# 루트 윈도우를 생성
label = Label(window, text="Hello tkinter")	# 레이블 위젯을 생성
label.pack()					# 레이블 위젯을 윈도우에 배치

window.mainloop()				# 윈도우가 사용자 동작을 대기