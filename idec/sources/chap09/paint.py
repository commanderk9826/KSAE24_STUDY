##
#	이 프로그램은 페인트 프로그램을 구현한다. 
#
from tkinter import *
from tkinter.colorchooser import askcolor

DEFAULT_PEN_SIZE = 1.0
DEFAULT_COLOR = "black"

mode = "pen"
old_x = None
old_y = None
mycolor = DEFAULT_COLOR
erase_on = False

def use_pen():		# 펜 버튼이 선택되면 모드를 ”pen"으로 바꾼다. 
    global mode
    mode = "pen"
    
def use_brush():	# 브러쉬 버튼이 선택되면 모드를 ”brush"으로 바꾼다. 
    global mode
    mode = "brush"

def choose_color():	# 색상 버튼이 선택되면 사용자한테 색상을 요구한다. 
    global mycolor
    mycolor = askcolor(color=mycolor)[1]

def use_eraser():	# 지우개 버튼이 선택되면 모드를 ”erase"으로 바꾼다. 
    global mode
    mode = "erase"

def paint(event):	# 이전 점과 현재 점 사이를 직선으로 연결한다. 
    global var, erase_on, mode, old_x, old_y
    fill_color = 'white' if   mode == "erase" else mycolor
    if   old_x and old_y:
        canvas.create_line(old_x, old_y, event.x, event.y,
                           capstyle=ROUND, width=var.get(), fill=fill_color)
    old_x = event.x
    old_y = event.y

def reset(event):	# 사용자가 마우스 버튼에서 손을 떼면 이전 점을 삭제한다. 
    global old_x, old_y
    old_x, old_y = None, None

def clear_all():	# 캔버스에 그려진 모든 그림을 삭제한다. 
    global cansvas
    canvas.delete('all')
    
window = Tk()	
var = DoubleVar()	# 슬라이더와 연결될 객체를 생성한다. 

penButton = Button(window, text='펜', command=use_pen)
penButton.grid(row=0, column=0, sticky=W+E)

brushButton = Button(window, text='브러쉬', command=use_brush)
brushButton.grid(row=0, column=1, sticky=W+E)

colorButton = Button(window, text='색상선택', command=choose_color)
colorButton.grid(row=0, column=2, sticky=W+E)

eraseButton = Button(window, text='지우개', command=use_eraser)
eraseButton.grid(row=0, column=3, sticky=W+E)

clearButton = Button(window, text='모두삭제', command=clear_all)
clearButton.grid(row=0, column=4, sticky=W+E)

scale = Scale(window, variable=var, orient=VERTICAL)
scale.grid(row=1, column=5, sticky=N+S)

canvas = Canvas(window, bg='white', width=600, height=400)
canvas.grid(row=1, columnspan=5)

canvas.bind('<B1-Motion>', paint)
canvas.bind('<ButtonRelease-1>', reset)

window.mainloop()