##
#	이 프로그램은 프록시마 센토리까지 빛이 가는 시간을 계산한다. 
#
speed = 300000.0			# 빛의 속도
distance = 40000000000000.0		# 거리
secs = distance / speed		# 걸리는 시간, 단위는 초	
light_year = secs / (60.0*60.0*24.0*365.0)	# 광년으로 변환
print(light_year, "광년")