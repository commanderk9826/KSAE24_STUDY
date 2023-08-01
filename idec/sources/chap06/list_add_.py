import time
SIZE =50000

start_time = time.time()
mylist =[]
for i in range(SIZE):
    mylist = mylist +[i * i]
print("수행시간=", time.time()- start_time)

start_time = time.time()
mylist =[]
for i in range(SIZE):
    mylist.append(i * i)
print("수행시간=", time.time()- start_time)
