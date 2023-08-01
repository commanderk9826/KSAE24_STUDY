from sprite import Sprite

class Brick(Sprite):
    def __init__(self, canvas, x, y):
        self.width = 52
        self.height = 25
        item = canvas.create_rectangle(x - self.width / 2, y - self.height / 2,
                                       x + self.width / 2, y + self.height / 2,
                                       fill='yellow', tags='brick')
        super().__init__(canvas, item)

    # 벽돌과 공이 충돌하면 벽돌을 삭제한다. 
    def handle_collision(self):
            self.delete()

