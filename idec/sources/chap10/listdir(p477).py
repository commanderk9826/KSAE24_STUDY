import os

cwd = os.getcwd()
files = os.listdir()
for name in files :
	if os.path.isfile(name) :
		if name.endswith(".jpg") :
			print(name)
