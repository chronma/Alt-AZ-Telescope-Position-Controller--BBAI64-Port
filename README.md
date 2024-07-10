This is a port of Alt-AZ-Telescope-Position-Controller beaglebone black code to beaglebone AI64.

Some key changes.

BBAI64 does not allow for config pin instead a device tree overlay is used.

DTO set up.  The BBAI64 is finicky. I sturggled getting overlays to load even pre included ones.  

Start with this image https://www.beagleboard.org/distros/beaglebone-ai-64-rev-b1
before doing anything.
**Update APT**

apt update
apt upgrade
apt autoclean
apt autoremove

**Create, Compile, and Load your Overlay.  (put all pin configurations in your .dts file. once loaded it will overide changes to parent device tree mappings)**
copy your.dts file to  /opt/source/dtb-5.10-ti/src/arm64/overlays
In directory /opt/source/dtb-5.10-ti/  run:

sudo make clean
sudo make
sudo make install_arm64

In directory /boot/firmware/extlinux  run:

sudo nano extlinux.conf
uncomment and change the ftdoverlay entry to be your overlay file
 ftdoverlays /overlays/your.dtbo
Save & exit nano

sudo reboot

This should load your overlay. (carefull one mistake in the overlay can stop the boot process and you need to reflash)
check by running

sudo beagle-version |grep UBOOT
xxd: Connection timed out
UBOOT: Booted Device-Tree:[k3-j721e-beagleboneai64.dts]
_UBOOT: Loaded Overlay:[Steppercontrol.kernel]_

Steppercontrol.dtbo was loaded.

**Addafruit GPIO is not ported to BBAI64 **
Use gpiod instead.
Need to configure your pins in the device tree overlay.
some sources
https://pypi.org/project/gpiod/
https://forum.beagleboard.org/t/bb-ai64-gpio-basics/36032
https://github.com/brgl/libgpiod/blob/master/bindings/python/examples/watch_multiple_line_values.py

**Ada-fruit BLinka is required for coms with the adafruit sensors**
IT curently is broked for AI64 make the following changes.

pip3 install Adafruit-Blinka
pip3 install adafruit-circuitpython-lsm303agr-mag
pip3 install adafruit-circuitpython-lsm303-accel
pip3 install adafruit-circuitpython-lis2mdl
pip3 install adafruit-circuitpython-mma8451

I have supporting ai64 header files in the repository, but you will still need to edit several headers.

this is how i did it (its not clean, sorry its been over 30 years since i've been in a programming class)

the adafruit attempt piggybacks on DRA74X chip id (BBAI?)  to attempt to keep this open for use i created pinai64.py instead of editing the pin.py files.

Under  /usr/local/lib/python3.9/dist-packages

edit digitalio.py and add the following elif statement. (I put it under the DRA74X elif entry)

elif detector.chip.TDA4VM:
    from adafruit_blinka.microcontroller.dra74x.pinai64 import *

edit board.py and add the following elif statement. (I put it under the BBAI elif entry)

elif board_id == ap_board.BEAGLEBONE_AI64:
    from adafruit_blinka.board.beagleboard.beaglebone_ai64 import *
    print("bbai64 select") #comment to troublshoot

under /usr/local/lib/python3.9/dist-packages/adafruit_platformdetect

edit board.py and add the following elif statement. (I put it under the BBAI elif entry)

there is a a try: statement near the bottom for the ai-64, this is trying to read 2-00500/nvmem this isn't right. 
#        try:
        #            # Special Case for AI64
        #            with open("/sys/bus/nvmem/devices/2-00500/nvmem", "rb") as eeprom:
        #                eeprom_bytes = eeprom.read(16)
        #        except FileNotFoundError:
        #            return None

instead use the get_device_model() method to obtain the board. 
# beagle,j721e-beagleboneai64^@ti,j721e^@

I added following abothe the memory read try: statements.

board_value = self.detector.get_device_model()
        if "BeagleBone AI-64" in board_value:
            return boards.BEAGLEBONE_AI64
            print ("board detected: ",board_value)

under /usr/local/lib/python3.9/dist-packages/microcontroller

edit pin.py and add the following elif statement. (I put it under the DRA74X elif entry)

elif chip_id == ap_chip.TDA4VM:
    from adafruit_blinka.microcontroller.dra74x.pinai64 import *

edit __init__.py and add the following elif statement. (I put it under the DRA74X elif entry)

elif chip_id == ap_chip.TDA4VM:
    from adafruit_blinka.microcontroller.dra74x import *

under /usr/local/lib/python3.9/dist-packages/adafruit_blinka/microcontroller/dra74x create(copy) pinai64.py

https://github.com/adafruit/Adafruit_Blinka/issues/687

pinai64.py probably needs work, but it is corrected for the Ai64 gpio address (ex. 1, 20 for P8.3) refer to 
https://docs.beagleboard.io/latest/boards/beaglebone/ai-64/04-expansion.html#bbai64-expansion
I couldn't find the address for the AI64 USRx LED's so they are not used.
pin.py under dra74x is for something else and all adresses are wrong.

# I2C4 need to change all to I2C2. 
I2C2_SCL = P9_19  # was i2c4_scl
I2C2_SDA = P9_20  # was i2c4_sda
make sure under  /usr/local/lib/python3.9/dist-packages/adafruit_blinka/microcontroller/dra74x
pinai64.py
order port for i2c2-5 has to match 5, I2C2  (not 3, i2c4)
i2cPorts = (
    (5, I2C2_SCL, I2C2_SDA),  # default config
    (5, I2C2_SCL, I2C2_SDA),  # roboticscape config
    (3, I2C5_SCL, I2C5_SDA),  # roboticscape config
)


under /usr/local/lib/python3.9/dist-packages/adafruit_blinka/board/beagleboard 

edit beaglebone_ai64.py (need to copy my file in as it has disappeared from blinka

change the from statement to:
from adafruit_blinka.microcontroller.dra74x import pinai64
instead of changing pin.xx_x 90 times i just added the following under the from statement: (i was tired)
pin=pinai64

Remember UART and I2C pins have to be configured in the device tree overlay.

**helpful commands**
ls -lah /dev/bone/i2c/
_should see 
lrwxrwxrwx 1 root root  11 Jun 18  2023 2 -> ../../i2c-5_
scan devices
i2cdetect -r 5
 the 5 is significant.

**PRU**

THe AI64 has more that just the 2 PRU's of the BBB.  IT was a painful experience to get PRU0 working.

Again the pins used need to be configured in the Device tree overlay. Do not select pins that are used for "boot select"
Comunication between the ARM and PRU is done by passing words back and forth via the SRAM of the PRU this maps to 0x0b000000 on the arm side. this was dificult to find.
the pru firmware executable is writen in C and compiled for the AI64 System on a chip (TDA4VM) this was done using TI's code composer studio.  you may have to change teh default memeory rqanges to get your program to compile.  In this repository is my CCS files and settings.
once compiled place your .out file in your home directory on the BBAI64 /home/debian/

echo 'stop' > /sys/class/remoteproc/remoteproc0/state
cp /home/debian//Steppercontrolpru2.out /lib/firmware/Steppercontrolpru2.out
echo 'Steppercontrolpru2.out' > /sys/class/remoteproc/remoteproc0/firmware
echo 'start' > /sys/class/remoteproc/remoteproc0/state

**Dependencies:**
Make sure yoru IDE is set to use Python 3, update apt and pip3
Ctypes (sudo pip3 install ctypes) install for all users
Adafruit Blinka (sudo pip3 install Adafruit-Blinka) isntall for all users
Pyserial (sudo pip3 install pyserial)
Numpy (sudo pip3 install numpy)
inputimeout 1.0.4 (sudo pip3 install inputimeout)
inquirer 3.1.3 (sudo pip3 install inquirer)
pynmeagps 1.0.29 (sudo pip3 install pynmeagps)
Skyfield (sudo pip3 install skyfield)
Flask 3.0.0 (sudo pip3 install Flask)
Indigo server #sudo indigo_server (kinda flakey)

Usage.
Home the scope and target something easy to find. set in auto mode and fine tune the position with the key pad or web page. once locked on a target it should switch to other targets fairly accurately.






















