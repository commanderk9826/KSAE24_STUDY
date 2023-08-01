##
#	클래스 간의 HAS-A 관계를 구현한다. 
#
class Card:
    suitNames = ['클럽', '다이아몬드', '하트', '스페이드']
    rankNames = [None, '에이스', '2', '3', '4', '5', '6', '7', 
              '8', '9', '10', '잭', '퀸', '킹']

    def __init__(self, suit, rank):
        self.suit = suit
        self.rank = rank

    def __str__(self):
        return Card.suitNames[self.suit]+" "+\
                             Card.rankNames[self.rank]

class Deck:
    def __init__(self):
        self.cards = []
        for suit in range(4):
            for rank in range(1, 14):
                card = Card(suit, rank)
                self.cards.append(card)
    def __str__(self):
        lst = [str(card) for card in self.cards]
        return str(lst)

deck = Deck()		# 덱 객체를 생성한다. 
print(deck)		# 덱 객체를 출력한다. __str__()이 호출된다. 