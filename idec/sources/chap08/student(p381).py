class Student:
	def __init__(self, name=None, age=0):
		self.__name = name		# __가 변수 앞에 붙으면 외부에서 변경 금지
		self.__age = age			# __가 변수 앞에 붙으면 외부에서 변경 금지

obj=Student()
print(obj.__age)			# 오류 발생!