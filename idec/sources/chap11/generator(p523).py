def MyCounterGen(low, high):
    while low <= high:
       yield low
       low += 1

for i in MyCounterGen(1, 10):
    print(i, end=' ')