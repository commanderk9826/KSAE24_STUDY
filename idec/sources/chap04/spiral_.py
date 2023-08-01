##
#	이 프로그램은 터틀 그래픽으로 스파이럴을 그린다. 
#
import turtle

t = turtle.Turtle()
t.shape("turtle")

for i in range(200):
    t.forward(2+i/4)		# 반복에 따라 조금씩 증가시킨다. 
    t.left(30-i/12)		# 반복에 따라 조금씩 감소시킨다. 	

turtle.mainloop()
turtle.bye()
