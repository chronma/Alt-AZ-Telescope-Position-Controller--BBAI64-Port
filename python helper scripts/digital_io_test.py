import time
import gpiod
from gpiod.line import Direction, Value

def get_line_value(chip_path, line_offset):
    with gpiod.request_lines(
        chip_path,
        consumer="get-line-value",
        config={line_offset: gpiod.LineSettings(direction=Direction.INPUT)},
    ) as request:
        value = str(request.get_value(line_offset))
        #print("{}={}".format(line_offset, value))
        #print(value)
        if (value=="Value.ACTIVE"):
            return 1
        else:
             return 0    
        


gpiod.is_gpiochip_device("/dev/gpiochip1")


PIN=71
while 1:
    p8_27=get_line_value("/dev/gpiochip1", PIN)
    print (p8_27,end="")
    time.sleep(.25)
    
