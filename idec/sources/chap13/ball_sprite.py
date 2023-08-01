from sprite import Sprite
from brick_sprite import Brick

class Ball(Sprite):
    def __init__(self, canvas, x, y, radius):
        self.radius = radius
        item = canvas.create_oval(x-self.radius, y-self.radius,
                                  x+self.radius, y+self.radius,
                                  fill='red')
        self.x = x
        self.y = y
        super().__init__(canvas, item)		# 부모 클래스 생성자 호출

    def update(self):
        x, y = self.get_position()
        width = self.canvas.winfo_width()
 		 
        # 벽에 부딪히면 방향을 변경한다. 
        if x <= 0 or x >= width:
            self.speedx *= -1		# x 방향 변경
        if y <= 0:
            self.speedy *= -1		# y 방향 변경

    def collide(self, obj_list):
        x, y = self.get_position()
        
        # 공이 패들이나 벽돌에 맞으면 y방향을 반대로 한다. 
        if len(obj_list):
            self.speedy *= -1

        for obj in obj_list:
            if isinstance(obj, Brick):
                obj.handle_collision()
