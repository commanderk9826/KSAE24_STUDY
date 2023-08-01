class Counter:
    def __init__(self) :
        self.count = 0
    def increment(self):
        self.count += 1
    def __str__(self):
        msg = "카운트값:"+ str(self.count)
        return msg

a = Counter(100)	
print(a)