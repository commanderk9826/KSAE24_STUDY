# 텔레비전을 클래스로 정의한다. 
class Television:
	def __init__(self, channel, volume, on):
		self.channel = channel
		self.volume = volume
		self.on = on

	def show(self):
		print(self.channel, self.volume, self.on)

# 전달받은 텔레비전의 음량을 줄인다. 
def setSilentMode(t):
	t.volume = 2

# setSilentMode()을 호출하여서 객체의 내용이 변경되는지를 확인한다. 
myTV = Television(11, 10, True);
setSilentMode(myTV)
myTV.show()