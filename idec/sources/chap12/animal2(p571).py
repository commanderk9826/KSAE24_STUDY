class Animal(object):
    pass

class Dog(Animal):
    def __init__(self, name):
        self.name = name

class Person(object):
    def __init__(self, name):
        self.name = name
        self.pet = None

dog1 = Dog("dog1")
person1 = Person("홍길동")
person1.pet = dog1