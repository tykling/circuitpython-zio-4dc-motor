# circuitpython-zio-4dc-motor
Circuitpython library for the Zio 4DC Motor Controller. Forked/ported from https://github.com/ZIOCC/Qwiic_4-Ch_DC_Motor_Controller/tree/master/micropython

## Usage
```python
import time
from zio_4dc_motor import Zio4DCMotor

# initialise i2c
i2c = board.STEMMA_I2C()
i2c.try_lock()
print(f"i2c bus devices: {str([hex(x) for x in i2c.scan()])}")
i2c.unlock()

# initialise and reset Zio 4DC motor controllers
motor = Zio4DCMotor(i2c=i2c, addr=0x40)

# turn off all motors
motor.reset()

# also define the ALLCALL device to control all the motor controllers on the bus
ALLCALL = Zio4DCMotor(i2c=i2c, addr=112)


# wrap main loop in try/except so we always disable all motors when the code exits
try:
    # the main loop
    while True:
        for mid in [0,1,2,3]:
            motor.go_forward(mid)
            motor.speed(mid, 50)
        time.sleep(5)
        ALLCALL.stop()
        for mid in [0,1,2,3]:
            motor.go_backward(mid)
            motor.speed(mid, 50)
        time.sleep(5)
        ALLCALL.stop()
        time.sleep(5)
finally:
    logger.warning("Code stopped, resetting (setting speed of all motors to 0 and disabling both drivers on all motor controllers)...")
    ALLCALL.reset()
    logger.error("Done, exiting.")
```
