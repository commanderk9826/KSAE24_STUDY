from tkinter import *

window = Tk()
T = Text(window, height=5, width=80)
T.pack()
T.insert(END, "테스트 위젯은 여러 줄의\n텍스트를 표시할 수 있습니다.")

window.mainloop()