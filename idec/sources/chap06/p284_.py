prices = [135, -545, 922, 356, -992, 217]
mprices = [i if i > 0 else 0 for i in prices]

words = ["All", "good", "things", "must", "come", "to", "an", "end."]
letters = [ w[0] for w in words ]

numbers = [x+y for x in ['a','b','c'] for y in ['x','y','z']]
