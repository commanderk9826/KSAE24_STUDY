import pickle

file = open( "save.p", "rb" )		# 이진 파일 오픈
obj = pickle.load( open( "save.p", "rb" ) )	# 피클 파일에 딕션너리를 로딩
print(obj)