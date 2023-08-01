##
#	이 프로그램은 메소드 오버라이딩을 구현한다. 
#
class Employee:
    def __init__(self, name, salary):
        self.name = name
        self.salary = salary
        
    def  getSalary(self):
        return self.salary

class Manager(Employee):
    def __init__(self, name, salary, bonus):
        super().__init__(name, salary)
        self.bonus =bonus

    def  getSalary(self):
        salary = super().getSalary()
        return salary + self.bonus

    def __str__(self):
        return "이름: "+ self.name+ "; 월급: "+ str(self.salary)+\
               "; 보너스: "+str(self.bonus)

kim = Manager("김철수", 2000000, 1000000)
print(kim)