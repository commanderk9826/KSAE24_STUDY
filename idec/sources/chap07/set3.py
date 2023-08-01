engineers =set(['Kim','Park','Lee'])
programmers =set(['Kim','Song','Choi'])
managers =set(['Chun','Seo','Oh'])

for group in [engineers, programmers, managers]: 
	group.discard('Kim') 		# 모든 그룹에서 “Kim"을 삭제한다.  
	print(group)