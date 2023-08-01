# 텔레비전을 클래스로 정의한다. 
class Television:
    serialNumber = 0		# 이것이 클래스 변수이다. 

    def __init__(self, channel, volume, on):
        self.channel = channel
        self.volume = volume
        self.on = on
        Television.serialNumber += 1		# 클래스 변수를 하나 증가한다. 
		# 클래스 변수의 값을 객체의 시리얼 번호로 한다. 
        self.number = Television.serialNumber
    
    def show(self):
        print(self.channel, self.volume, self.on, self.number)

myTV = Television(11, 10, True);
myTV.show()