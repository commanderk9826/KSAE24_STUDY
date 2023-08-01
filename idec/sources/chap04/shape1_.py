import turtle
t = turtle.Turtle()
t.shape("turtle")

s = turtle.textinput("", "몇각형을 원하시나요?:")
n=int(s)

for i in range(n):
    t.forward(100)
    t.left(360/n)

turtle.mainloop()
turtle.bye()
