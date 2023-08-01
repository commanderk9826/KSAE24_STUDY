class Sprite():
    def __init__(self, canvas, item):
        self.canvas = canvas	# 캔버스 객체
        self.item = item		# 캔버스 안에 있는 도형의 식별 번호
        self.speedx = 3			# x 방향 속도
        self.speedy = 3			# y 방향 속도
        self.x = 0				# 현재 x좌표  
        self.y = 0				# 현재 x좌표 

	  # 도형의 위치와 크기를 반환한다. 
    def get_coords(self):
        return self.canvas.coords(self.item)

	  # 도형의 위치를 반환한다. 
    def get_position(self):
        pos = self.canvas.coords(self.item)
        x = pos[0]
        y = pos[1]
        return x, y

	  # 객체의 상태를 변경한다. 
    def update(self):
        self.x = self.x + self.speedx
        self.y = self.y + self.speedy

	  # 객체를 움직인다. 
    def move(self):
        self.canvas.move(self.item, self.speedx, self.speedy)

	  # 객체를 캔버스에서 삭제한다. 
    def delete(self):
        self.canvas.delete(self.item)

