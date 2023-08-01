from tkinter import *
import random
  
class Car(Canvas): 
    def __init__(self, master = None):
        Canvas.__init__(self, master, width = 500, height = 100)
        self.master = master 
        self.dx = 0
        self.dy = 0
        self.rectangle = self.create_rectangle(0, 5, 70, 35, fill = "black") 
        self.oval = self.create_oval(5, 20, 25, 50, fill = "black") 
        self.oval2 = self.create_oval(40, 20, 60, 50, fill = "black") 
        self.pack() 
        self.movement() 
      
    def movement(self): 
        dx = random.randint(2, 10)
        self.move(self.rectangle, dx, 0) 
        self.move(self.oval, dx, 0) 
        self.move(self.oval2, dx, 0) 
        self.after(100, self.movement) 
      
master = Tk() 
car1 = Car(master) 
car2 = Car(master) 
  
mainloop()