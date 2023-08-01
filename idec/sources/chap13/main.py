##
#	이 프로그램은 벽돌 깨기 게임을 구현한다.  
#
from tkinter import * 
from ball_sprite import Ball
from brick_sprite import Brick
from paddle_sprite import Paddle

class BrickBreaker(Frame):
    def __init__(self, root):
        super().__init__(root)
        self.width =  640
        self.height = 480 
        self.canvas = Canvas(self, bg='blue',
                                width=self.width,
                                height=self.height,)
        self.canvas.pack()
        self.pack()
        
        # shapes에는 화면에 있는 모든 객체가 저장된다. 
        # 키는 도형 식별 번호이고 값은 객체이다. 
        self.shapes = {}

        # 패들 객체를 생성하고 shapes에 저장한다. 
        self.paddle = Paddle(self.canvas, self.width/2, 450)
        self.shapes[self.paddle.item] = self.paddle

        # Ball 객체를 생성한다. 
        self.ball = Ball(self.canvas, 310, 200, 10)

        # Brick 객체를 2차원 모양으로 생성한다. 
        for r in range(1, 4):
            for c in range(1, 10):
                brick = Brick(self.canvas, c*60, r*30)
                # Brick 객체를 shapes에 저장한다. 
                self.shapes[brick.item] = brick

        # 캔버스가 키보드 이벤트를 받을 수 있도록 설정한다. 
        self.canvas.focus_set()
        # 화살표키와 스페이스키에 이벤트를 붙인다.
        self.canvas.bind('<Left>',
                         lambda _: self.paddle.move(-10, 0))
        self.canvas.bind('<Right>',
                         lambda _: self.paddle.move(10, 0))
        self.canvas.bind('<space>', lambda _: self.start())

    def start(self):
        self.game_loop()

    # 게임 루프를 작성한다. 
    def game_loop(self):
        coords = self.ball.get_coords() # Ball 객체의 위치를 구한다. 
        # 겹치는 모든 도형을 찾는다. 식별 번호가 저장된다. 
        items = self.canvas.find_overlapping(*coords)
        
        # 겹치는 도형의 식별 번호로 객체를 찾아서 리스트에 저장한다. 
        objects = [self.shapes[x] for x in items if x in self.shapes]
        
        # 충돌 처리 메소드를 호출한다. 
        self.ball.collide(objects)
        self.ball.update()
        self.ball.move()
        
        # game_loop()를 50밀리초 후에 호출한다. 
        self.after(50, self.game_loop)

window = Tk()
game = BrickBreaker(window)
window.mainloop()
