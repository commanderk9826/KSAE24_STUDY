class Counter:
    def __init__(self,  initValue=0) :
        self.count = initValue

    def increment(self) :
        self.count += 1

a = Counter(0)			# 계수기를 0으로 초기화한다. 
b = Counter(100)		# 계수기를 100으로 초기화한다. 