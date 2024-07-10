import busio
from board import *
from adafruit_bus_device.i2c_device import I2CDevice
i2c = busio.I2C(SCL, SDA)
i2c.try_lock()
print(i2c.scan())
i2c.unlock()
i2c.deinit()


with busio.I2C(SCL, SDA) as i2c:
    device = I2CDevice(i2c, 0x1d)
    bytes_read = bytearray(64)
    with device:
      device.readinto(bytes_read)
      # A second transaction
    with device:
        device.write(bytes_read)
