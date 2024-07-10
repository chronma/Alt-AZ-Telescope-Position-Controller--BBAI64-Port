import sys

for x in range(10):
    print("Progress {:2.1%}".format(x / 10), end="\b")

for x in range(10):
    sys.stdout.write('\r'+str(x))
    sys.stdout.flush()