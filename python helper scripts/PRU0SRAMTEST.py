import ctypes
from ctypes import *

import os
import mmap
import time

# need to run script from command line as sudo,sudo python3 ctypestest.py
Task= ['getbit','setbit', 'clearbit']

def bitmagic(value, bitnumber,task):
    if task == 'getbit':
        return ((value>>bitnumber)&1)
    if task == 'setbit':
        return (value | (1<<bitnumber))
    if task == 'clearbit':
        return (value & ~(1<<bitnumber))
    
    if task =='TogleBit':
        return (value ^((value>>bitnumber)&1))
    
a=0x00000
b=0x0b000000
c=(a+b)
print (c)

a=0x0000
b=0x0b000000
d=(a+b)
print (hex(c),hex(d))




fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=c)
os.close(fd)

fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem1 = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=d)
os.close(fd)

#v = ctypes.c_uint32.from_buffer(mem, 0x28).value
#t=v
#for i in range (11,12):
#        t=bitmagic(t,i,'clearbit')
        #t=bitmagic(t,i,'setbit')

#ctypes.c_uint32.from_buffer(mem, 0x28).value = t
rto=1
while (rto):
    v = ctypes.c_uint32.from_buffer(mem, 0x0).value
    w = ctypes.c_uint32.from_buffer(mem, 0x130).value
    x = ctypes.c_uint32.from_buffer(mem, 0x134).value
    y = ctypes.c_uint32.from_buffer(mem, 0x138).value
    
#  print ('incoming',v,'  ',t)
 #   for i in range (0x16):
 #       xd = mem[i], mem[i+1]
 #       print (xd)

    print("w0 ", end=' ')
    for i in range (31,-1,-1):
        print(bitmagic(v,i,'getbit'), end='')
    print(' ')
    print("w1 ", end=' ')
    for i in range (31,-1,-1):

        print(bitmagic(w,i,'getbit'), end='')
    print(' ')
    print("w2 ", end=' ')
    for i in range (31,-1,-1):
        print(bitmagic(x,i,'getbit'), end='')
    print(' ')
    print("w3 ", end=' ')
    for i in range (31,-1,-1):
        print(bitmagic(y,i,'getbit'), end='')
    print(' ')
    
    #for i in range (0,31):
#        t=bitmagic(t,i,'clearbit')
        #t=bitmagic(t,i,'setbit')

    #ctypes.c_uint32.from_buffer(mem, 0x28).value = t 

#    print ('out',t,'  ','')
#    for i in range (0,31):
#        print(bitmagic(t,i,'getbit'), end='')
#    print(' ')
#    print ('verified bits',v,'  ','')
#    for i in range (0,31):
#        print(bitmagic(v,i,'getbit'), end='')
#    print(' ')
    time.sleep (.25)
    rto=1



    