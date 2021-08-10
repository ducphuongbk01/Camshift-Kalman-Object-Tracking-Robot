import numpy as np

a = 32
a = str(a)
a = str.encode(a)
a = list(a)
if (len(a)<3):
    a.insert(0,0x30)
print(a)