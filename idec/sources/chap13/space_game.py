# pygame 라이브러리를 포함한다. 
import pygame

# pygame 라이브러리를 초기화한다. 
pygame.init()

WIDTH = 600
HEIGHT = 400

# 그림이 그려지는 화면 설정
mydisplay = pygame.display.set_mode([WIDTH, HEIGHT])
pygame.display.set_caption('Shooting Game')

black = (0,0,0)
white = (255,255,255)
blue = (0,0,255)
spaceshipImage = pygame.image.load('spaceship.png')
saucerImage = pygame.image.load('saucer.png')

class SpaceShip(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.image = pygame.image.load('spaceship.png')
        self.dx = 1
        self.dy = 1
        self.rect = self.image.get_bounding_rect()
        self.rect.x = 100
        self.rect.y = 100
        
    def move(self, dx, dy):
        self.rect.x += dx
        self.rect.y += dy
    
class EnemyShip(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.image = pygame.image.load('saucer.png')
        self.dx = -1
        self.dy = 0
        self.rect = self.image.get_bounding_rect()
        self.rect.x = 500
        self.rect.y = 300

    def move(self):
        self.rect.x += self.dx
        if self.rect.x < 0:
            self.rect.x = 500
            
# 사용자가 중단할 때까지 반복 실행한다. 
x = 100
y = 200
dx = 0
dy = 0

# 모든 객체를 하나의 리스트에 저장하여 관리한다. 
object_list = pygame.sprite.Group()

ship = SpaceShip()
object_list.add(ship)

enemy = EnemyShip()
object_list.add(enemy)

running = True
frame_no = 0
while running:
    frame_no += 1
    # 사용자가 중단 버튼을 눌렀으면 
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    key = pygame.key.get_pressed()
    if key[pygame.K_UP]:
        ship.move(0, -1)
    if key[pygame.K_DOWN]:
        ship.move(0, 1)
    if (frame_no % 10) == 0:
        enemy.move()
    
    # 배경을 휜색으로 채운다. 
    mydisplay.fill(white)
    # 모든 스프라이트를 화면에 그린다. 
    for obj in object_list:
        mydisplay.blit(obj.image, (obj.rect.x, obj.rect.y))
    
    pygame.display.update()

    # 충돌을 감지한다. 
    if pygame.sprite.spritecollideany(ship, [enemy]):
        # 충돌이 발생하였으면 주인공을 소멸시키고 게임을 종료한다. 
        ship.kill()
        running = False

# 종료한다. 
pygame.quit()
