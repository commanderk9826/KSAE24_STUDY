##
#	이 프로그램은 움직이는 원을 화면에 그린다. 
#
import tkinter as tk
import time

CANVAS_WIDTH = 600
CANVAS_HEIGHT = 400
 
dx = 3		# 원의 x 방향 속도이다. 
dy = 3		# 원의 y 방향 속도이다. 

window = tk.Tk()
canvas = tk.Canvas(window, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg='lightblue')
canvas.pack()

# 캔버스에 원을 생성한다. 
obj_id = canvas.create_oval(50,0,150,100, fill="red")
# 애니메이션을 그리는 반복 루프
while True:
    canvas.move(obj_id, dx, dy)
    x0, y0, x1, y1 = canvas.coords(obj_id)	# 현재 위치를 얻는다. 
    if   y1 > canvas.winfo_height() or y0 < 0:	# 원이 위쪽이나 아래쪽으로 벗어났으면
        dy = -dy				# dy의 부호를 반전시킨다. 
    if   x1 > canvas.winfo_width() or x0 < 0:	# 원이 왼쪽이나 오른쪽으로 벗어났으면
        dx = -dx				# dx의 부호를 반전시킨다. 
    window.update()				# 화면을 업데이트한다. 
    time.sleep(0.05)				# 약간의 시간지연 

window.mainloop()