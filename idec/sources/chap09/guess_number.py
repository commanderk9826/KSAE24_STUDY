from tkinter import *
import random

answer = random.randint(1,100)		# 정답을 1에서 100 사이의 난수로 설정한다. 

def guessing():
	guess = int(guessField.get())	# 텍스트 필드에서 사용자가 입력한 값을 가져온다.

	if   guess > answer:
		msg = "높음!"
	elif   guess < answer:
		msg = "낮음!"
	else:
		msg = "정답!"
		
	resultLabel["text"] = msg		# 메시지를 출력한다. 
	guessField.delete(0, 5)

def reset():				# 정답을 다시 설정한다. 
	global answer
	answer = random.randint(1,100)
	resultLabel["text"] = "다시 한번 하세요!"

window = Tk()
window.configure(bg="white")
window.title("숫자를 맞춰보세요!")
window.geometry("500x80")

titleLabel = Label(window, text="숫자 게임에 오신 것을 환영합니다!", bg="white")
titleLabel.pack()

guessField = Entry(window)
guessField.pack(side="left")
tryButton = Button(window, text="시도", fg="green", bg="white", command=guessing )
tryButton.pack(side="left")

resetButton = Button(window, text="초기화", fg="red", bg="white", command=reset)
resetButton.pack(side="left")
resultLabel = Label(window, text="1부터 100사이의 숫자를 입력하시오.", bg="white") 
resultLabel.pack(side="left")

window.mainloop()