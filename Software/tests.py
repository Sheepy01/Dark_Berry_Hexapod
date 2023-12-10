import time

chars = 'ABCDEFGH'
loop = range(1, len(chars) + 1)

LINE_CLEAR = '\x1b[2K' # <-- ANSI sequence

for idx in loop:
    print(chars[:idx], end='\r')
    time.sleep(.5)

print(end=LINE_CLEAR) # <-- clear the line where cursor is located
print('done')