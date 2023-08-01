import math

class Shape:
     def __init__(self):
        pass

     def draw(self):
        print("draw()가 호출됨")

     def get_area(self):
        print("get_area()가 호출됨")

class Circle(Shape):
     def __init__(self, radius=0):
        super().__init__()
        self.radius = radius
 
     def draw(self):
        print("원을 그립니다.")

     def get_area(self):
        return math.pi * self.radius ** 2 

c = Circle(10)
c.draw()
print("원의 면적:", c.get_area())