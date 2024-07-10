import gpiod
import time
from gpiod.line import Edge


def edge_type_str(event):
    if event.event_type is event.Type.RISING_EDGE:
        return "Rising"
    if event.event_type is event.Type.FALLING_EDGE:
        return "Falling"
    return "Unknown"


def watch_multiple_line_values(chip_path, line_offsets):
    with gpiod.request_lines(
        chip_path,
        consumer="watch-multiple-line-values",
        config={tuple(line_offsets): gpiod.LineSettings(edge_detection=Edge.BOTH)},
    ) as request:
        while True:
            if (request.wait_edge_events(.1)):
                print("input detected: ")
                for event in request.read_edge_events():
                    print(
                        "offset: {}  type: {:<7}  event #{}  line event #{}".format(
                            event.line_offset,
                            edge_type_str(event),
                            event.global_seqno,
                            event.line_seqno,
                    ))
                    return event.line_offset
            else:
                return 0            
            
                


in1=watch_multiple_line_values("/dev/gpiochip1", [71, 72, 73])
while 1:
    in1=watch_multiple_line_values("/dev/gpiochip1", [71, 72, 73])
    time.sleep(.25)
    print(in1,end=" ")