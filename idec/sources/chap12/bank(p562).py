class Bank(): 
	def getInterestRate(self):
		return 0.0

class BadBank(Bank):
	def getInterestRate(self):
		return 10.0;

class NormalBank(Bank):
	def getInterestRate(self):
		return 5.0;

class GoodBank(Bank):
	def getInterestRate(self):
		return 3.0;

b1 = BadBank()
b2 = NormalBank()
b3 = GoodBank()
print("BadBank의 이자율: " + str(b1.getInterestRate()))
print("NormalBank의 이자율: " + str(b2.getInterestRate()))
print("GoodBank의 이자율: " + str(b3.getInterestRate()))
