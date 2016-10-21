import re
mystring = 'a4f3'
valids = re.sub(r"[^1-9]+", '', mystring)
print valids