# 사용자로부터 상품의 가격을 입력받는다.  
price = int(input("상품의 가격: "))

# 배송비를 결정한다. 
if  price > 20000 :
	shipping_cost = 0
else :
	shipping_cost = 3000

# 배송비를 출력한다. 
print("배송비 = ", shipping_cost)