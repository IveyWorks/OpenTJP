# MicroPython test script for Raspberry Pi Pico and MLX90614 temperature sensor

# MLX90614 test script, version 2025-01-12

# """
# MicroPython MLX90614 IR temperature sensor driver
# https://github.com/mcauser/micropython-mlx90614
# 
# MIT License
# Copyright (c) 2016 Mike Causer
# Copyright (c) 2025 Craig Ivey
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# """






# I2C1
# GPIO Pin 19 = Pico pin 25  (I2C1, SCL)
# GPIO Pin 18 = Pico pin 24  (I2C1, SDA)
_I2C_Channel = 1
_I2C_GPIO_SCL_Pin = 19
_I2C_GPIO_SDA_Pin = 18
_I2C_Address = 0x5a



import ustruct

class SensorBase:

    def read16(self, register):
        data = self.i2c.readfrom_mem(self.address, register, 2)
        return ustruct.unpack('<H', data)[0]

    def read_temp(self, register):
        temp = self.read16(register);
        # apply measurement resolution (0.02 degrees per LSB)
        temp *= .02;
        # Kelvin to Celcius
        temp -= 273.15;
        return temp;

    def read_ambient_temp(self):
        return self.read_temp(self._REGISTER_TA)

    def read_object_temp(self):
        return self.read_temp(self._REGISTER_TOBJ1)

    def read_object2_temp(self):
        if self.dual_zone:
            return self.read_temp(self._REGISTER_TOBJ2)
        else:
            raise RuntimeError("Device only has one thermopile")

class MLX90614(SensorBase):

    _address = _I2C_Address

    _REGISTER_TA = 0x06
    _REGISTER_TOBJ1 = 0x07
    _REGISTER_TOBJ2 = 0x08

    def __init__(self, i2c, address=_address):
        self.i2c = i2c
        self.address = address
        _config1 = i2c.readfrom_mem(address, 0x25, 2)
        _dz = ustruct.unpack('<H', _config1)[0] & (1<<6)
        self.dual_zone = True if _dz else False


def C2F(inC):
    return inC * 9./5. + 32.


def TestForMLX90614(i2c):
    addr = _I2C_Address
    devlist = i2c.scan()
    for item in devlist:
        if item == addr: return True
    return False


import time
#import mlx90614
from machine import I2C, Pin

led = Pin("LED", Pin.OUT)


i2c = I2C(_I2C_Channel, scl=Pin(_I2C_GPIO_SCL_Pin), sda=Pin(_I2C_GPIO_SDA_Pin), freq=100000)
time.sleep_ms(200)

counter = 0

while not TestForMLX90614(i2c):
    print('Counter:', counter)
    print('Did not find a connected MLX90614 at assumed address and pins')
    print('       _I2C_Channel', _I2C_Channel)
    print('  _I2C_GPIO_SCL_Pin', _I2C_GPIO_SCL_Pin)
    print('  _I2C_GPIO_SDA_Pin', _I2C_GPIO_SDA_Pin)
    print('       _I2C_Address', '0x{:02x}'.format(_I2C_Address))
    counter += 1
    time.sleep_ms(500)



time.sleep(1)

sensor = MLX90614(i2c)

led.value(1)  # LED ON

counter = 0

while True:
    counter += 1
    try:  #  if an error occurs during I2C connection, then handle gracefully
        RAC = sensor.read_ambient_temp()
        RAF = C2F(RAC)
        ROC = sensor.read_object_temp()
        ROF = C2F(ROC)
        print(counter, 'Ambient {:.0f} C  {:.0f} F    Object {:.0f} C  {:.0f} F'.format(RAC, RAF, ROC, ROF))
    except: # catch all errors in here and not crash
        print('caught error')
    led.toggle()
    time.sleep_ms(500)