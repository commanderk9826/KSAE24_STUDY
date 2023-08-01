price = int(input("정가를 입력하시오: "))
if price >= 100 :
	dis_rate = 0.85
	print("10층에서 사은품을 받아가세요.")
else :
	dis_rate = 0.90
dis_price = dis_rate * price
print("할인된 상품의 가격=", dis_price)
