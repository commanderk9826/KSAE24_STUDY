text="""101 COM   PythonProgramming
102 MAT   LinearAlgebra
103 ENG   ComputerEnglish"""

import re
s = re.findall("\d+", text)
print(s)