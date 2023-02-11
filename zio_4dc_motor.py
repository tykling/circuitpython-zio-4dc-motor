"""
CircuitPython driver for Zio 4DC Motor Controller based on PCA9685 and TB6612.

Based on/ported from: https://github.com/ZIOCC/Qwiic_4-Ch_DC_Motor_Controller/tree/master/micropython

product link: https://www.smart-prototyping.com/Zio-4-DC-Motor-Controller.html

PCA9685PW datasheet: https://www.smart-prototyping.com/image/data/NOA-RnD/101897%204%20DC%20Motor%20Driver/PCA9685PW_datasheet.pdf

TB6612FNG datasheet: https://www.smart-prototyping.com/image/data/NOA-RnD/101897%204%20DC%20Motor%20Driver/TB6612FNG_datasheet.pdf

-----------------PCA8685 PINOUT--------------------
|  PCA9685  --  TB6612       PCA9685  --   TB6612 |
---------------------------------------------------
|  0        --  PWMA1         8       --   A2IN1  |
|  1        --  A1IN1         9       --   A2IN2  |
|  2        --  A1IN2         10      --   STBY2  |
|  3        --  STBY1         11      --   B2IN1  |
|  4        --  B1IN1         12      --   B2IN2  |
|  5        --  B1IN2         13      --   PWMB2  |
|  6        --  PWMB1         14      --   IO14   |
|  7        --  PWMA2         15      --   IO15   |
---------------------------------------------------

The motor terminals are labelled with:
- A101/A102 for motor 0 (driven by TB6612 #1)
- B101/B102 for motor 1 (driven by TB6612 #1)
- A201/A202 for motor 2 (driven by TB6612 #2)
- B201/B202 for motor 3 (driven by TB6612 #2)

Example: Run motor 0 clockwise at 50% speed for 5s:

    import board
    import time
    from zio_4dc_motor import Zio4DCMotor
    i2c = board.STEMMA_I2C() # or whatever i2c you use
    controller = Zio4DCMotor(i2c=i2c, addr=0x40) # address may vary
    controller.go_forward(0) # set direction
    controller.speed(0, 50) # set speed 50% for motor 0
    time.sleep(5)
    # use controller.stop() to stop all motors at once
    controller.speed(0, 0) # set speed to 0 for motor 0

"""

import struct
import time

from adafruit_bus_device.i2c_device import I2CDevice

# register addresses
PCA9685_MODE1 = 0x00  # < Mode Register 1
PCA9685_LED0_ON_L = 0x06  # < LED0 on tick, low byte

# register values
PCA9685_PRESCALE = 0xFE  # < Mode Register 1
MODE1_SLEEP = 0x10  # < Low power mode. Oscillator off
MODE1_RESTART = 0x80  # < Restart with previous settings


class Zio4DCMotor:
    """Control a single Zio 4DC Motor Controller. Also supports ALLCALL for controlling multiple."""

    def __init__(self, i2c, addr: int = 0x40, freq: int = 1000, restart: bool = False):
        """Initilisation code. Define useful buffers and constants."""
        self.buf1 = bytearray(1)
        self.buf4 = bytearray(4)
        self.addr = addr

        # initialise I2CDevice object
        self.device = I2CDevice(i2c, addr)

        # a tuple of tuples of: (speed pin, clockwise pin, counterclockwise pin, tb6612)
        self.dc_motor = ((0, 1, 2, 1), (6, 4, 5, 1), (7, 8, 9, 2), (13, 11, 12, 2))

        # the pins used to turn each BB6612 driver on/off
        self.tb6612pins = {1: 3, 2: 10}

        # set the frequency
        self.set_freq(freq)

        # do we want to restart or reset?
        if restart:
            # start with everything in their last known state
            self.restart()
        else:
            # start with everything turned off
            self.reset()
            # but enable both drivers so they are ready for use
            self.enable_tb6612(1)
            self.enable_tb6612(2)

    def read(self, reg: int, bufsize: int = 1):
        """Read bufsize bytes from the device, starting at register reg. Retry until it works."""
        buf = bytearray(bufsize)
        while True:
            with self.device:
                try:
                    self.device.write_then_readinto(
                        out_buffer=bytearray([reg]), in_buffer=buf
                    )
                except OSError as E:
                    print(
                        f"Got an exception while reading {bufsize} bytes from i2c register {reg}: {E} .... retrying!"
                    )
            return buf

    def write(self, reg: int, buf: bytearray):
        """Write the bytearray to the register on the device. Retry until it works."""
        while True:
            with self.device:
                try:
                    self.device.write(bytearray([reg]) + buf)
                    break
                except OSError as E:
                    print(
                        f"Got an exception while writing to i2c register {reg}: {E} .... retrying!"
                    )

    def set_pwm(self, num: int, on: int, off: int):
        """Calculate register and write a two-byte buffer to set PWM for the two pins for motor num."""
        # PCA9685_LED0_ON_L is the first/lowest register, we calculate from there
        reg = PCA9685_LED0_ON_L + 4 * num
        self.buf4[0:2] = struct.pack("<H", on)
        self.buf4[2:4] = struct.pack("<H", off)
        self.write(reg=reg, buf=self.buf4)

    def restart(self):
        """Restart the 9685 in the state it had before going to sleep."""
        self.send_command8(reg=PCA9685_MODE1, cmd=MODE1_RESTART)
        time.sleep(0.00001)

    def send_command8(self, reg: int, cmd: int = 0):
        """Write a 1 byte command passed as an integer."""
        self.buf1[0] = cmd
        self.write(reg=reg, buf=self.buf1)

    def set_freq(self, freq: int):
        """Set the pwm frequency."""
        # calculate prescale from freq
        prescale = int(25000000.0 / 4096.0 / freq + 0.5)
        # remember the current mode1
        old_mode = int(self.read(0x00)[0])
        # go to sleep (since prescale can only be set while sleeping)
        self.send_command8(reg=0x00, cmd=(old_mode & 0x7F) | MODE1_SLEEP)
        # set prescale
        self.send_command8(reg=PCA9685_PRESCALE, cmd=prescale)
        # go back to the previous mode
        self.send_command8(reg=PCA9685_MODE1, cmd=old_mode)
        # it takes up to 500↓s after waking up from sleep
        time.sleep(0.000005)
        # enable register auto-increment
        self.send_command8(
            reg=PCA9685_MODE1, cmd=old_mode | 0xA1
        )  # Same mode 1 but with autoincrement on

    def set_pin(self, num: int, val: int, invert=False):
        """Set PWM value for pins. These are 12 bit registers so the max value is 4095."""
        val = min(val, 4095)
        if invert:
            if val == 0:
                self.set_pwm(num, 4096, 0)
            elif val == 4095:
                self.set_pwm(num, 0, 4096)
            else:
                self.set_pwm(num, 0, 4095 - val)
        else:
            if val == 4095:
                self.set_pwm(num, 4096, 0)
            elif val == 0:
                self.set_pwm(num, 0, 4096)
            else:
                self.set_pwm(num, 0, val)

    def enable_tb6612(self, driver: int = 1):
        """Enable a TB6612 by setting pin 3 or 10 high."""
        self.set_pin(num=self.tb6612pins[driver], val=0, invert=True)

    def disable_tb6612(self, driver: int = 1):
        """Disable a TB6612 by setting pin 3 or 10 low."""
        self.set_pin(num=self.tb6612pins[driver], val=0, invert=False)

    def stop(self):
        """Set all motor speeds to 0."""
        # fastest way to stop all motors is to write logic 1 to bit 4 of register 0xFD
        self.send_command8(reg=0xFD, cmd=0b00010000)

    def reset(self):
        """Stop both tb6612 drivers and set all motor speeds to 0."""
        # stop all motors
        self.stop()
        # also disable both drivers
        self.disable_tb6612(1)
        self.disable_tb6612(2)

    def go_forward(self, motor: int):
        """Start the motor in a clockwise direction."""
        self.set_pin(self.dc_motor[motor][1], 0, True)
        self.set_pin(self.dc_motor[motor][2], 4095, True)

    def go_backward(self, motor: int):
        """Start the motor in a counter-clockwise direction."""
        self.set_pin(self.dc_motor[motor][1], 4095, True)
        self.set_pin(self.dc_motor[motor][2], 0, True)

    def speed(self, motor: int, speed: int):
        """Set the speed of the motor from 0% to 100%."""
        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0
        pulselen = int(speed / 100 * 4095)
        pinlist = self.dc_motor[motor]
        self.set_pwm(pinlist[0], 0, pulselen)
        time.sleep(0.000100)
