class Animal:
    def __init__(self, age=0):
        self.age=age

    def eat(self):
        print("동물이 먹고 있습니다. ")


class Dog(Animal):
    def __init__(self, age=0, name=""):
        self.name=name

d = Dog();
print(d.age)    # 부모 클래스의 생성자가 호출되지 않아서 오류가 발생한다. 