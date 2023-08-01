##
#	이 프로그램은 터틀 그래픽으로 랜덤 워크를 구현한다. 
#
import turtle
import random
t = turtle.Turtle()
t.shape("turtle")

for i in range(30):		
   length = random.randint(1, 100)
   t.forward(length)
   angle = random.randint(-180, 180)
   t.right(angle) 

turtle.mainloop()
turtle.bye()
