from sprite import Sprite

class Paddle(Sprite):
    def __init__(self, canvas, x, y):
        self.width = 100 
        self.height = 20
        item = canvas.create_rectangle(x - self.width / 2, y - self.height / 2,
                                       x + self.width / 2, y + self.height / 2,
                                       fill='white')
        super().__init__(canvas, item)  # 부모 클래스 생성자 호출
        self.x = x                      # 현재 위치 저장
        self.y = y

    # 패들을 dx, dy만큼 이동한다. 키보드 이벤트에서 호출된다. 
    def move(self, dx, dy):
        self.x = self.x + dx
        self.y = self.y + dy
        self.canvas.move(self.item, dx, dy)
