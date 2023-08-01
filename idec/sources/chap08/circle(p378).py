import math

# Circle 클래스를 정의한다. 
class Circle:
    def __init__(self, radius = 0):
        self.radius = radius

    def getArea(self):
        return  math.pi * self.radius * self.radius

    def getPerimeter(self):
        return 2 * math.pi * self.radius 

# Circle 객체를 생성한다. 
c = Circle(10)
print("원의 면적", c.getArea())
print("원의 면적", c.getPerimeter())
