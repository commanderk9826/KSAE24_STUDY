class Dog:
    kind = "Bulldog"         	# 모든 인스턴스 변수들이 공유하는 클래스 변수
    def __init__(self, name, age=0):
        self.name = name    	# 각 인스턴스에 유일한 인스턴스 변수
        self.age = age    	# 각 인스턴스에 유일한 인스턴스 변수

a = Dog("Baduk", 2)
b = Dog("Marry", 3)

print(a.kind)                  # 모든 강아지가 공유
print(b.kind)                  # 모든 강아지가 공유
print(Dog.kind)                # 모든 강아지가 공유

print(a.name)                  # 강아지 a에 유일
print(b.name)                  # 강아지 b에 유일