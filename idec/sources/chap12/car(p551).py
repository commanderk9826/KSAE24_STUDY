# 일반적인 자동차를 나타내는 클래스이다. 
class Car:
	def __init__(self, make, model, color,  price):
		self.make = make			# 메이커
		self.model = model		# 모델
		self.color = color		# 자동차의 색상
		self.price = price		# 자동차의 가격

	def setMake(self, make):	# 설정자 메소드 
		self.make = make

	def getMake(self):		# 접근자 메소드
		return self.make

	# 차량에 대한 정보를 문자열로 요약하여서 반환한다.
	def getDesc(self):
		return "차량 =("+str(self.make)+","+\
                                        str(self.model)+","+\
                                        str(self.color)+","+\
                                        str(self.price)+")"
class ElectricCar(Car) :				# ①
	def __init__(self, make, model, color, price, batterySize):
		super().__init__(make, model, color, price)	# ②
		self.batterySize=batterySize			# ③

	def setBatterySize(self,  batterySize):	# 설정자 메소드 
		self.batterySize=batterySize

	def getBtterySize(self):			# 접근자 메소드 
		return self.batterySize
def main():						# main() 함수 정의
	myCar = ElectricCar("Tisla", "Model S", "white", 10000, 0)	# 객체 생성
	myCar.setMake("Tesla")				# 설정자 메소드 호출
	myCar.setBatterySize(60)				# 설정자 메소드 호출
	print(myCar.getDesc())				# 트럭 객체를 문자열로 출력

main()