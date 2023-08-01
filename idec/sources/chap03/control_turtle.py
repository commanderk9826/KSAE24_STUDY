import turtle

# 거북이를 만든다.
t = turtle.Turtle()

# 커서의 모양을 거북이로 한다. 
t.shape("turtle")

# 거북이가 그리는 선의 두께를 3으로 한다. 
t.width(3)

# 거북이를 3배 확대한다. 
t.shapesize(3, 3)

# 무한 루프이다. 
while True: 
    command = input("명령을 입력하시오: ")
    if command == "l":		# 사용자가 “l“을 입력하였으면
        t.left(90)
        t.forward(100)
    if command == "r":		# 사용자가 “r“을 입력하였으면
        t.right(90)
        t.forward(100)
    if command == "q":		# 사용자가 “q“을 입력하였으면
        break			# 무한 루프를 빠져나간다. 

turtle.mainloop()
turtle.bye()