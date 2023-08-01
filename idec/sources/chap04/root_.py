COUNT = 100
START = 1.0
END = 2.0

for i in range(COUNT):
    x = START + i*((END-START)/COUNT)
    f = (x**2 -x -1)
    if abs(f-0.0) < 0.01 :
        print("방정식의 해는 ", x)
