import ctypes
from ctypes import *
import subprocess
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
    
a=0x0000000
b=0x0b000000
c=(a+b)
print (c)

a1steps=1000
a1pulselength=int(8964143*.1)

a2steps=1000
a2pulselength=int(8964143*.5)

a3steps=1000000
a3pulselength=int(8964143*.0000075)

fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=c)
os.close(fd)

command_bits = ctypes.c_uint32.from_buffer(mem, 0x200).value # 0x00 is the command bit

t=0
    
t=bitmagic(t,0,'setbit')
t=bitmagic(t,1,'setbit')
t=bitmagic(t,2,'setbit')
t=bitmagic(t,3,'setbit')
t=bitmagic(t,4,'setbit')
t=bitmagic(t,5,'setbit')
print (hex(t))
print (time.asctime())
for x in range(0,1,1):
    ctypes.c_uint32.from_buffer(mem, 0x100).value = a1steps
    ctypes.c_uint32.from_buffer(mem, 0x104).value = a1pulselength 
    ctypes.c_uint32.from_buffer(mem, 0x108).value = a2steps
    ctypes.c_uint32.from_buffer(mem, 0x10C).value = a2pulselength 
    ctypes.c_uint32.from_buffer(mem, 0x110).value = a3steps
    ctypes.c_uint32.from_buffer(mem, 0x114).value = a3pulselength 
    
    
    ctypes.c_uint32.from_buffer(mem, 0x000).value = t 
    

    command_bits = ctypes.c_uint32.from_buffer(mem, 0x000).value # 0x00 is the command bit
    print (hex(command_bits))
    
    h=0
    

    for x in range (0,3):
        if (bitmagic(command_bits,x,"getbit")==1):
            h=bitmagic(h,x,'setbit')
            
        else:    
            h=bitmagic(h,x,'clearbit')
            
    print(hex(h))
    
    while ((h!=0)):
        time.sleep(.001)
        
        command_bits = ctypes.c_uint32.from_buffer(mem, 0x000).value # 0x00 is the command bit
        
        for x in range (0,3):
            if (bitmagic(command_bits,x,"getbit")==1):
                h=bitmagic(h,x,'setbit')
                
            else:    
                h=bitmagic(h,x,'clearbit')
                
        
    time.sleep(5)
       
t=0
ctypes.c_uint32.from_buffer(mem, 0x000).value = t 
