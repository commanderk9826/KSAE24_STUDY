counter = [0] *26
infile = open("mobydick.txt", "r")
ch = infile.read(1)
while ch != "" :
    ch = ch.upper()		# 대문자->소문자
    if ch >= "A" and ch <= "Z" :
        i = ord(ch) - ord("A")
        counter[i] += 1
    ch = infile.read(1)
print(counter)