class Parent(object): 
       def __init__(self): 
              self.__money = 100

class Child(Parent): 
       def __init__(self): 
              super().__init__() 

obj = Child() 
print(obj.money) # 오류 