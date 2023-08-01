try:
   f = open("test.txt", "w" )
   f.write("테스트 데이터를 파일에 씁니다!!")
   ...				# 파일 연산을 수행한다. 
except IOError:
   print("Error: 파일을 찾을 수 없거나 데이터를 쓸 수 없습니다. ")
finally:
   f.close()