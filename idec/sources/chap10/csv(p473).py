import csv

f = open('weather.csv')			# CSV 파일을 열어서 f에 저장한다. 
data = csv.reader(f)
header = next(data)
temp = 1000
for row in data:
    if temp > float(row[3]):
        temp = float(row[3])
print('가장 추웠던 날은', temp, '입니다')
f.close()