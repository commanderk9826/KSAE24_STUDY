##
#	이 프로그램은 사각형을 그리는 함수를 작성하고 사용한다. 
#

import turtle 
t = turtle.Turtle()
t.shape("turtle")

def square(length):		# length는 한변의 길이
    t.down()
    for i in range(4):
        t.forward(length)
        t.left(90)
    t.up()

square(100);			# square() 함수를 호출한다. 
t.forward(120)
square(100);
t.forward(120)
square(100);

turtle.mainloop()
turtle.bye()	
