class Student:
	def __init__(self, name=None, age=0):
		self.__name = name
		self.__age = age

	def getAge(self):
		return self.__age

	def getName(self):
		return self.__name

	def setAge(self, age):
		self.__age=age

	def setName(self, name):
		self.__name=name

obj=Student("Hong", 20)
obj.getName()