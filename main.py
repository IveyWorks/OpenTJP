# OpenTJP tracker with MLX90614 temperature sensor
# Release 2025-01-12a

MY_CHANNEL = 599
MY_CALLSIGN = 'N2QST'
MY_BAND = '20m'       # ['10m', '12m', '15m', '17m', '20m', '30m', '40m', '80m', '160m']  remember to include the 'm' character

MY_ET_ENABLE = True
MY_GEOFENCE_ENABLE = True
MY_DEBUG_PRINT_ENABLE = True


# BSD 3-Clause License
# 
# Copyright (c) 2025, Craig Ivey
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



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




import os
import sys
import machine
import math
from machine import SPI, Pin, I2C, UART, mem32, WDT
import SI5351
import time
#import pyb


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



global clockgen
global uart1
global sensor
global PicoTemp
global AmbientTemp
global ObjectTemp

#isUSBconnected = bool(machine.mem32[0x50110000 + 0x50] & (1<<16))
MY_RMC_VALID = False
MY_GGA_QUALITY = 0
MY_LAT = None
MY_LON = None
MY_ALT = None
MY_SPEED = None
MY_MM = None
MY_SS = None
MY_POWER = 13
MY_CHANNEL_TIME_MM = -1
MY_CHANNEL_LANE = -1
MY_TEMPERATURE = -99
MY_VOLTAGE = -99
MY_GRID = None
MY_SIGGENPARAMS = None
WARMUP_PERIOD_SEC = 30
IS_CONNECTED = True
MY_ET_SLOT = 2
#MY_BIGNUM = 0
MY_BAND = MY_BAND.lower()

MY_BIGCOUNTER = 0

SYNC = [1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0, 
    0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0, 
    1,1,0,0,0,1,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,
    1,1,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,0,0]

SIGGENPARAMS = {
    # Band         Symbol 0              Symbol 1              Symbol 2              Symbol 3
    '10m':[ [[15, 8, 17927, 3056], [15, 8, 28797, 4909], [15, 8, 31865, 5432], [15, 8, 13193, 2249]],   # Freq lane 1,  20 Hz
            [[15, 8, 13498, 2301], [15, 8, 17223, 2936], [15, 8, 23183, 3952], [15, 8, 35103, 5984]],   # Freq lane 2,  60 Hz
            [[15, 8, 24403, 4160], [15, 8, 32727, 5579], [15, 8, 27119, 4623], [15, 8, 27776, 4735]],   # Freq lane 3, 140 Hz
            [[15, 8, 10776, 1837], [15, 8, 18836, 3211], [15, 8, 47703, 8132], [15, 8, 12747, 2173]]]   # Freq lane 4, 180 Hz
    ,
    '12m':[ [[15, 8, 58823, 7693], [15, 8, 18397, 2406], [15, 8, 11263, 1473], [15, 8, 24277, 3175]],
            [[15, 8, 67876, 8877], [15, 8, 26999, 3531], [15, 8, 12624, 1651], [15, 8, 23497, 3073]],
            [[15, 8, 135667, 17743], [15, 8, 44264, 5789], [15, 8, 21570, 2821], [15, 8, 13465, 1761]],
            [[15, 8, 17311, 2264], [15, 8, 12578, 1645], [15, 8, 25653, 3355], [15, 8, 110327, 14429]]]
    ,
    '15m':[ [[15, 8, 39630, 3779], [15, 8, 40427, 3855], [15, 8, 26815, 2557], [15, 8, 32415, 3091]],
            [[15, 8, 17167, 1637], [15, 8, 72653, 6928], [15, 8, 12773, 1218], [15, 8, 6785, 647]],
            [[15, 8, 21047, 2007], [15, 8, 32163, 3067], [15, 8, 31366, 2991], [15, 8, 2779, 265]],
            [[15, 8, 34061, 3248], [15, 8, 21781, 2077], [15, 8, 19799, 1888], [15, 8, 14650, 1397]]]
    ,
    '17m':[ [[15, 8, 31128, 2299], [15, 8, 24155, 1784], [15, 8, 22625, 1671], [15, 8, 21095, 1558]],
            [[15, 8, 33890, 2503], [15, 8, 34743, 2566], [15, 8, 3236, 239], [15, 8, 29977, 2214]],
            [[15, 8, 25265, 1866], [15, 8, 63677, 4703], [15, 8, 14853, 1097], [15, 8, 7000, 517]],
            [[15, 8, 11793, 871], [15, 8, 17263, 1275], [15, 8, 30938, 2285], [15, 8, 129398, 9557]]]
    ,
    '20m': [ [[15, 8, 31445, 1599], [15, 8, 104561, 5317] , [15, 8, 20865, 1061], [15, 8, 36440, 1853]],   # Freq lane 1,  20 Hz
             [[15, 8, 54237, 2758], [15, 8, 19685, 1001], [15, 8, 34375, 1748], [15, 8, 24503, 1246]],     # Freq lane 2,  60 Hz
             [[15, 8, 44109, 2243], [15, 8, 70421, 3581], [15, 8, 21966, 1117], [15, 8, 13156, 669]],   # Freq lane 3, 140 Hz
             [[15, 8, 12566, 639], [15, 8, 37639, 1914] , [15, 8, 45918, 2335], [15, 8, 74983, 3813]]  ] # Freq lane 4, 180 Hz
    ,         
    '22m': [ [[15, 8, 37097, 1786], [15, 8, 126890, 6109], [15, 8, 66301, 3192], [15, 8, 30928, 1489]], 
            [[15, 8, 27563, 1327], [15, 8, 80965, 3898], [15, 8, 41978, 2021], [15, 8, 31551, 1519]], 
            [[15, 8, 51823, 2495], [15, 8, 45571, 2194], [15, 8, 26545, 1278], [15, 8, 103189, 4968]], 
            [[15, 8, 39215, 1888], [15, 8, 16845, 811], [15, 8, 45010, 2167], [15, 8, 34957, 1683]]]
    ,
    '30m': [[[15, 8, 26227, 861], [15, 8, 20744, 681], [15, 8, 56353, 1850], [15, 8, 55561, 1824]], 
            [[15, 8, 82275, 2701], [15, 8, 65887, 2163], [15, 8, 26897, 883], [15, 8, 34299, 1126]], 
            [[15, 8, 18185, 597], [15, 8, 80081, 2629], [15, 8, 28237, 927], [15, 8, 68841, 2260]], 
            [[15, 8, 28907, 949], [15, 8, 50869, 1670], [15, 8, 52788, 1733], [15, 8, 41609, 1366]]] 
    ,
    '40m': [[[15, 8, 133519, 2817], [15, 8, 39103, 825], [15, 8, 42800, 903], [15, 8, 15499, 327]], 
            [[15, 8, 52137, 1100], [15, 8, 3460, 73], [15, 8, 55123, 1163], [15, 8, 58346, 1231]], 
            [[15, 8, 45169, 953], [15, 8, 30950, 653], [15, 8, 36448, 769], [15, 8, 58677, 1238]], 
            [[15, 8, 43889, 926], [15, 8, 48913, 1032], [15, 8, 56449, 1191], [15, 8, 53700, 1133]]]
    ,
    '80m': [[[15, 8, 29563, 292], [15, 8, 40396, 399], [15, 8, 62062, 613], [15, 8, 36245, 358]], 
            [[15, 8, 147307, 1455], [15, 8, 34321, 339], [15, 8, 52747, 521], [15, 8, 33916, 335]], 
            [[15, 8, 31283, 309], [15, 8, 7188, 71], [15, 8, 33409, 330], [15, 8, 30878, 305]], 
            [[15, 8, 30169, 298], [15, 8, 40799, 403], [15, 8, 64185, 634], [15, 8, 153477, 1516]]]
    ,
    '160m':[[[15, 8, 79428, 389], [15, 8, 29811, 146], [15, 8, 18785, 92], [15, 8, 100663, 493]], 
            [[15, 8, 41857, 205], [15, 8, 12455, 61], [15, 8, 55537, 272], [15, 8, 110053, 539]], 
            [[15, 8, 66764, 327], [15, 8, 69214, 339], [15, 8, 56147, 275], [15, 8, 8371, 41]], 
            [[15, 8, 703354, 3445], [15, 8, 157004, 769], [15, 8, 88404, 433], [15, 8, 61454, 301]]]

}


GEOFENCE_EXCL = ['PM27','PM28','PM29','PN20','PM37','PM38','PM39','PN30','PN31','PM48','PN40','PN41',
    'PN42','PN52','LK12','LK13','LK14','LK15','LK16','LK17','LK22','LK23','LK24','LK25',
    'LK26','LK27','LK33','LK34','LK35','LK36','LK37','LK44','LK45','LK46','LK47','LK48',
    'LK54','LK55','LK56','LK57','LK58','LK65','LK66','LK67','LK68','FM08']
    # N.Korea
    # Yemen
    # NRQZ





global PowerGPS
global PowerOsc
global led
global VBATGPS



def eprint(*args):
    if MY_DEBUG_PRINT_ENABLE:
        for val in args:
            print(val, end=' ')
        print('')


def encodeBigNumTemperatures():
    #PicoTemp, AmbientTemp, ObjectTemp

    out = 0
    
    x = int(PicoTemp)
    if x < -60:
        x = -60
    elif x > 59:
        x = 59
    x += 60
    out = out * 120 + x
    
    x = int(AmbientTemp)
    if x < -60:
        x = -60
    elif x > 59:
        x = 59
    x += 60
    out = out * 120 + x

    x = int(ObjectTemp)
    if x < -60:
        x = -60
    elif x > 59:
        x = 59
    x += 60
    out = out * 120 + x

    return out

    

def isGeofenced(test_grid):
    if not MY_GEOFENCE_ENABLE: return False
    if not isinstance(test_grid, str): return False
    if len(test_grid) < 4: return False
    testgrid4 = test_grid[0:4]
    for excluded_grid in GEOFENCE_EXCL:
        if len(excluded_grid)==4 and testgrid4 == excluded_grid:
            return True
        elif test_grid == excluded_grid:
            return True
    return False
    

def FeedWatchdog():
    pass


def isGPSDataValid():
    return MY_RMC_VALID and (MY_GGA_QUALITY > 0)


def restartUART():
    global uart1
    uart1 = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9))

def disableGPS():
    global PowerGPS
    
    # TODO: set UART pins to high Z
    
    # Turn off GPS
    NRESETGPS.value(0)  # 0 = reset
    time.sleep(0.1)
    PowerGPS.value(1)  # 1 = OFF
    time.sleep(0.1)


def enableGPS():
    # TODO: enable UART pins after a high-Z state
    
    global PowerGPS
    global PowerOsc
    eprint('enableGPS')
    #FeedWatchdog()
    # wake up the GPS
    PowerGPS.value(0)  # 0 = ON
    time.sleep(0.1)
    NRESETGPS.value(1)  # 1 = not in reset state
    #time.sleep(0.5)


def disableOsc():
    global PowerGPS
    global PowerOsc
    #global led
    PowerOsc.value(1)  # 1 = OFF
    #led.value(0)

def enableOsc():
    global PowerGPS
    global PowerOsc
    global led
    global clockgen
    PowerOsc.value(0)  # 0 = ON
    #led.value(1)
    time.sleep(0.5)
    #devlist = i2c0.scan()
    #eprint(devlist)
    clockgen = SI5351.SI5351( i2c0) 
    clockgen.begin()
    clockgen.setClockBuilderData()
    PLL_mult = MY_SIGGENPARAMS[0][0]
    clockgen.setupPLL(PLL_mult, 0, 1, pllsource='A')


def hardReset():
    disableOsc()
    disableGPS()
    VBATGPS.value(0)   # 0 = disable battery backup power
    machine.reset() # Pico reset


def stopSymbol():
    global clockgen
    clockgen.enableOutputs(False)

def emitSymbol(SymbolNum):
    global clockgen
    DIV  = MY_SIGGENPARAMS[SymbolNum][1]
    NUM  = MY_SIGGENPARAMS[SymbolNum][2]
    DENOM= MY_SIGGENPARAMS[SymbolNum][3]
    clockgen.setupMultisynth( output=0, div=DIV, num=NUM, denom=DENOM, pllsource="A", phase_delay=0, inverted=False)
    clockgen.setupMultisynth( output=1, div=DIV, num=NUM, denom=DENOM, pllsource="A", phase_delay=0, inverted=True)
    clockgen.enableOutputs(True)


def LightSleepMs(delay_ms):
    machine.lightsleep(int(delay_ms))



def DelayUsDeadline(deadline):
    while time.ticks_diff(deadline, time.ticks_us()) > 0:
        #FeedWatchdog()
        time.sleep_us(10) # 0.010 ms

def DelayMsDeadlineAndFeed(deadline):
    counter = 0
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        counter += 1
        if counter >= 1000:
            counter = 0
            #FeedWatchdog()
        time.sleep_us(100) # 0.100 ms

def DelayMsAndFeed(delay_ms):
    if delay_ms <= 0: return
    deadline = time.ticks_add(time.ticks_ms(), int(delay_ms))
    counter = 0
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        counter += 1
        if counter >= 1000:
            counter = 0
            #FeedWatchdog()
        time.sleep_us(500) # 0.500 ms


def TxSymbol(count, symbol):
    #eprint(count, symbol)
    if symbol == '0':
        emitSymbol(0)
    elif symbol == '1':
        emitSymbol(1)
    elif symbol == '2':
        emitSymbol(2)
    elif symbol == '3':
        emitSymbol(3)
    else:
        eprint('symbol error')


def TxWSPRSequence(inArr):
    global led
    for i,c in enumerate(inArr):
        led.toggle()
        deadline = time.ticks_add(time.ticks_us(), 682667)  # 682667 us
        TxSymbol(i, c)
        #FeedWatchdog()
        DelayUsDeadline(deadline)
        stopSymbol()
    


def InvalidateGPS():
    global MY_RMC_VALID
    global MY_GGA_QUALITY
    MY_RMC_VALID = False
    MY_GGA_QUALITY = 0


def convertLatLonToGridSquare(Lat, Lon):

    A_float = (int(Lon + 180.)%360.) / 20
    A_int = int(A_float)
    A_char = chr( A_int + ord('A'))
    A_rem = (Lon + 180.)%360. - A_int*20
    
    B_int = int(A_rem / 2.)
    B_rem = (A_rem  ) - B_int*2.
    
    C_char = chr(int(B_rem * 12) + ord('A'))
    
    X_float = (int(Lat + 90.)%180.) / 10
    X_int = int(X_float)
    X_char = chr( X_int + ord('A'))
    X_rem = (Lat + 90.)%180. - X_int*10
    
    Y_int = int(X_rem)
    Y_rem = (X_rem) - Y_int
    
    Z_char = chr( int(Y_rem*24) + ord('A'))

    return A_char+X_char+str(B_int)+str(Y_int)+C_char+Z_char
    



def properlyFormatCallsign(inStr):
    # Callsigns in the format of:
    # PP7LLL
    # PP7LL
    #  P7LLL
    #  P7LL
    # PP7L
    
    inStr = inStr.strip()  # remove leading/trailing whitespace
    inStr = inStr.upper()  # uppercase
    
    # check for spaces in the middle
    testArr = inStr.split(' ')
    if len(testArr) > 1:
        return None
    
    
    len_inStr = len(inStr)
    if len_inStr > 6 or len_inStr < 4:
        return None
    
    #reverse_callsign = inStr[::-1]
    last_number_index = -1
    for i,c in enumerate(inStr):
        if c.isdigit():
            last_number_index = i
            
    
    outStr = ''
    
    if len_inStr == 6:
        # AB#CDE
        if last_number_index == 2:
            outStr = inStr
        else:
            return None
    elif len_inStr == 5:
        # A#CDE
        if last_number_index == 1:
            outStr = " " + inStr
        elif last_number_index == 2:
            # AB#DE
            outStr = inStr + " "
        else:
            return None
    elif len_inStr == 4:
        # A#AA
        if last_number_index == 1:
            outStr = " " + inStr + " "
        # AA#A
        elif last_number_index == 2:
            outStr = inStr + "  "
        else:
            return None
    else:
        return None
    
    
    
    
    if len(outStr) != 6:
        return None
        
    if not outStr[0].isspace() and not outStr[0].isdigit() and not outStr[0].isalpha():
        # 1st character is a space, letter, or number
        return None
    elif not outStr[1].isdigit() and not outStr[1].isalpha():
        # 2nd character is a letter or number
        return None
    elif not outStr[2].isdigit():
        # 3rd character is a number
        return None
    elif not outStr[3].isalpha():
        # 4th character is a letter
        return None
    elif not outStr[4].isspace() and not outStr[4].isalpha():
        # 5th character is a space or letter
        return None
    elif not outStr[5].isspace() and not outStr[5].isalpha():
        # 6th character is a space or letter
        return None
    
    return outStr
    
    
    

def convertCallsignToNumber(inStr):
    # Derived from: "The WSPR Coding Process", PDF file, Andy Talbot, G4JNT, June 2009.

    # N1 = [Ch 1] The first character can take on any of the 37 values including [sp],
    # N2 = N1 * 36 + [Ch 2] but the second character cannot then be a space so can have 36 values
    # N3 = N2 * 10 + [Ch 3] The third character must always be a number, so only 10 values are possible.
    # N4 = 27 * N3 + [Ch 4] – 10]
    # N5 = 27 * N4 + [Ch 5] – 10] Characters at the end cannot be numbers,
    # N6 = 27 * N5 + [Ch 6] – 10] so only 27 values are possible.
    # (In practice N will just be one 32 bit integer variable that can be built up in stages)



    N_array = [0, 0, 0, 0, 0, 0]
    
    for i,c in enumerate(inStr):
        N = 0
        if c == ' ':
            N = 36
        elif c.isdigit():
            N = ord(c) - 48
        else:
            N = ord(c.upper()) - 65 + 10
        
        N_array[i] = N

    N1 = N_array[0]
    N2 = N1 * 36 + N_array[1]
    N3 = N2 * 10 + N_array[2]
    N4 = N3 * 27 + N_array[3] - 10
    N5 = N4 * 27 + N_array[4] - 10
    N6 = N5 * 27 + N_array[5] - 10
    
    return N6
    
def convertLocator(inStr):
    # Derived from: "The WSPR Coding Process", PDF file, Andy Talbot, G4JNT, June 2009.
    Loc_array = [0, 0, 0, 0]
    
    Loc_array[0] = ord(inStr[0].upper()) - 65 #+ 10
    Loc_array[1] = ord(inStr[1].upper()) - 65 #+ 10
    Loc_array[2] = ord(inStr[2]) - 48
    Loc_array[3] = ord(inStr[3]) - 48
    
    # M1 = (179 - 10 * [Loc 1] - [Loc 3] ) * 180 + 10 * [Loc 2] + [Loc 4]

    M1 = (179 - 10 * Loc_array[0] - Loc_array[2] ) * 180 + 10 * Loc_array[1] + Loc_array[3]
    
    return M1
    
    
    


def encode(CALLSIGN, LOCATION, POWER):
    # Derived from: "The WSPR Coding Process", PDF file, Andy Talbot, G4JNT, June 2009.
    #M = M1 * 128 + [Pwr] + 64
    M = convertLocator(LOCATION) * 128 + POWER + 64
    N = convertCallsignToNumber(CALLSIGN)

    M_binary = "{:022b}".format(M)
    N_binary = "{:028b}".format(N)

    C = N_binary + M_binary + '0'*31  # pad 0 at the end to make 81 digits

    return C


def findParity(REG):
    p = 0
    while REG:
        p ^= REG&1
        REG >>= 1
    return p





def convolutionalCoding(C):
    # Derived from: "The WSPR Coding Process", PDF file, Andy Talbot, G4JNT, June 2009.

    CW = '0'*31 + C   # pre-pad 0 at the beginning, effectively "clears" the REGs

    MAGIC0 = int('F2D05351', 16) # Magic numbers from an obscure 1970 IEEE paper.
    MAGIC1 = int('E4613C47', 16)

    S = []
    for i in range(0, 81):
        
        REG0_binary_string = REG1_binary_string = CW[i:i+32]
        
        REG0 = int(REG0_binary_string, 2) & MAGIC0
        REG1 = int(REG1_binary_string, 2) & MAGIC1
        
        S.append(findParity(REG0))
        S.append(findParity(REG1))
        
    return S


def interleave(S):
    # Derived from: "The WSPR Coding Process", PDF file, Andy Talbot, G4JNT, June 2009.
    # Initialise a counter, P to zero
    # Take each 8-bit address from 0 to 255, referred to here as I
    # Bit-reverse I to give a value J.
    # For example, I = 1 gives J = 128, I = 13 J = 176 etc.
    # If the resulting bit-reversed J yields a value less than 162 then :
        # Set Destination bit D[J] = source bit S[P]
        # Increment P
    # Stop when P = 162

    D = [None]*162
    P = 0
    for I in range(0,256):
        J = '{:08b}'.format(I)
        #J = J[::-1]  # reverse string, doesn't work in MicroPython
        J = "".join(reversed(J))  # reverse string
        J = int(J, 2)  # convert string of binary digits to an integer
        if J < 162:
            D[J] = S[P]
            P += 1
        if P >= 162: break
    return D





    
def mergeWithSync(D):
    # Derived from: "The WSPR Coding Process", PDF file, Andy Talbot, G4JNT, June 2009.
    global SYNC
    #Symbol[n] = Sync[n] + 2 * Data[n]
    Symbol = [None]*162
    for i,Data in enumerate(D):
        Symbol[i] = SYNC[i] + 2*Data
    return Symbol
    
    



def GetChannelLane(Channel):
    Channel = int(Channel)
    if Channel < 0:
        Channel = 0
    elif Channel > 599:
        Channel = 599

    return (int(Channel/5) % 4) + 1



def GetChannelTime(Channel):
    global MY_BAND
    # band    10  12  15  17  20  30  40  80  160
    # time     4,  0,  6,  2,  8,  4,  0,  2,  8
    # offset   3   0   2   4   1   3   0   4   1  
    
    offset = 1
    
    if MY_BAND == '10m': offset = 3
    elif MY_BAND == '12m': offset = 0
    elif MY_BAND == '15m': offset = 2
    elif MY_BAND == '17m': offset = 4
    elif MY_BAND == '20m': offset = 1
    elif MY_BAND == '30m': offset = 3
    elif MY_BAND == '40m': offset = 0
    elif MY_BAND == '80m': offset = 4
    elif MY_BAND == '160m': offset = 1

    Channel = int(Channel)
    if Channel < 0:
        Channel = 0
    elif Channel > 599:
        Channel = 599
    
    return ((Channel-offset) % 5) * 2




def EncodeExtTelemetry(Channel, BigNum):
    # Infinite thanks to Doug.
    
    eprint('BigNum', BigNum)

    Channel = int(Channel)
    if Channel < 0:
        Channel = 0
    elif Channel > 599:
        Channel = 599


    CALLSIGN = [None]*6

    CALLSIGN[0] = ''
    if 0 <= Channel and Channel <= 199:
        CALLSIGN[0] = '0'
    elif 200 <= Channel and Channel <= 399:
        CALLSIGN[0] = '1'
    elif 400 <= Channel and Channel <= 599:
        CALLSIGN[0] = 'Q'
    else:
        #error
        eprint('bad channel', Channel)
        return None
        

    CALLSIGN[2] = chr(  int(Channel / 20) % 10 + 48  )


    #36
    C_1 = BigNum / 1900
    C_1 = C_1 / (18*18 * 26*26*26)
    MSW_char_index = int(C_1) % 36
    if MSW_char_index < 10:
        MSW_char = chr( MSW_char_index + 48 )
    else:
        MSW_char = chr( MSW_char_index - 10 + 65 )
    CALLSIGN[1] = MSW_char
    
    #26
    C_3 = BigNum / 1900
    C_3 = C_3 / (18*18 * 26*26)
    CSW_char_index = int(C_3) % 26
    CSW_char = chr( CSW_char_index + 65 )
    CALLSIGN[3] = CSW_char
    
    #26
    C_4 = BigNum / 1900
    C_4 = C_4 / (18*18 * 26)
    LSW_char_index = int(C_4) % 26
    LSW_char = chr( LSW_char_index + 65 )
    CALLSIGN[4] = LSW_char
    
    
    #26
    C_5 = BigNum / 1900
    C_5 = C_5 / (18*18)
    DSW_char_index = int(C_5) % 26
    DSW_char = chr( DSW_char_index + 65 )
    CALLSIGN[5] = DSW_char
    
    #18
    A = BigNum / 1900
    A = A / (18)
    #A_char_index = int(A / 34200) % 18
    A_char_index = int(A) % 18
    A_char = chr(  A_char_index + 65 )
    
    
    #18
    B = BigNum // 1900   # integer division
    B_char_index = B % 18
    B_char = chr(  B_char_index + 65 )
    
    
    
    #10
    C = BigNum % 1900
    #eprint('Ca',C)
    C = C / (10 * 19)
    #eprint('Cb:',C)
    C_char_index = int(C) % 10
    C_char = chr( C_char_index + 48 )
    
    
    
    #10
    D = BigNum % 1900
    #eprint('Da:',D)
    D = D / (19)
    #eprint('Db:',D)
    D_char_index = int(D) % 10
    D_char = chr( D_char_index + 48 )
    
    #19
    E = BigNum % 19
    E_char_index = E
    POWERS = [0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60]
    E_val = POWERS[E_char_index]



    out = [CALLSIGN[0] + \
        CALLSIGN[1] + \
        CALLSIGN[2] + \
        CALLSIGN[3] + \
        CALLSIGN[4] + \
        CALLSIGN[5], \
        A_char + B_char + C_char + D_char, \
        E_val]

    eprint(out[0], out[1], out[2])

    return out[0], out[1], out[2]


def EncodeExtTelemValues(HdrTelemetryType, HdrRESERVED, HdrType, HdrSlot, ValueArr):
    
    # Infinite thanks to Doug.
    out = 0
    #for val in ValueArr:
    out = ValueArr[0]
    
    # HdrSlot
    out = out * 5
    out = out + HdrSlot
    
    #HdrType
    out = out * 16
    out = out + HdrType   # should be 0
    
    #HdrRESERVED
    out = out * 4
    out = out + HdrRESERVED   # should be 0
    
    #HdrTelemetryType = 0
    out = out * 2
    out = out + HdrTelemetryType  #  should be 0
    
    return out



def EncodeU4B(Channel, Locator56, Altitude, Temperature, Millivolts, Speed, GPSValid):
    # Derived from spreadsheet 308d.xls (http://www.qrp-labs.com/images/ultimate3builder/ve3kcl/s4/308d.xls)
    
    Channel = int(Channel)
    if Channel < 0:
        Channel = 0
    elif Channel > 599:
        Channel = 599

    Temperature = int(Temperature)
    if Temperature < -50:
        Temperature = -50
    elif Temperature > 39:
        Temperature = 39
    
    
    Millivolts = int(Millivolts)
    if Millivolts < 3000:
        Millivolts = 3000
    elif Millivolts > 4950:
        Millivolts = 4950
        
    Speed = int(Speed)
    if Speed < 0:
        Speed = 0
    elif Speed > 82:
        Speed = 82


    Altitude = int(Altitude)
    if Altitude < 0:
        Altitude = 0
    elif Altitude > 21340:
        Altitude = 21340


    if GPSValid:
        GPSValid = 1
    else:
        GPSValid = 0


    CALLSIGN = [None]*6

    CALLSIGN[0] = ''
    if 0 <= Channel and Channel <= 199:
        CALLSIGN[0] = '0'
    elif 200 <= Channel and Channel <= 399:
        CALLSIGN[0] = '1'
    elif 400 <= Channel and Channel <= 599:
        CALLSIGN[0] = 'Q'
    else:
        #error
        eprint('bad channel', Channel)
        return None
        
    if 65 <= ord(Locator56[0]) and ord(Locator56[0]) <= 90:
        # good
        pass
    else:
        eprint('bad locator5', Locator56[0])
        return None
        
    if 65 <= ord(Locator56[1]) and ord(Locator56[1]) <= 90:
        # goood
        pass
    else:
        eprint('bad locator6', Locator56[1])
        return None
        



    CALLSIGN[2] = chr(  int(Channel / 20) % 10 + 48  )



    Locator5_ord = ord(Locator56[0]) - 65  #  24 values
    Locator6_ord = ord(Locator56[1]) - 65  #  24 values
    LocatorBits_ord = 24*Locator5_ord + Locator6_ord #  576 values

    Alt_index = round((Altitude+.1) / 20) # adding .1 to fix Python rounding #  1068 values

    MSW = 1068*LocatorBits_ord + Alt_index
    MSW_char_index = int( MSW / (26*26*26) )
    if MSW_char_index < 10:
        MSW_char = chr( MSW_char_index + 48 )
    else:
        MSW_char = chr( MSW_char_index - 10 + 65 )

    CSW = MSW - (MSW_char_index * (26*26*26))
    CSW_char_index = int( CSW / (26*26) )
    CSW_char = chr( CSW_char_index + 65 )


    LSW = CSW - (CSW_char_index * (26*26))
    LSW_char_index = int( LSW / (26) )
    LSW_char = chr( LSW_char_index + 65 )

    DSW = LSW - (LSW_char_index * (26))
    DSW_char_index = DSW
    DSW_char = chr( DSW_char_index + 65 )


    CALLSIGN[1] = MSW_char
    CALLSIGN[3] = CSW_char
    CALLSIGN[4] = LSW_char
    CALLSIGN[5] = DSW_char

        
    POWERS = [0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60]

        
    Temp_int = int(Temperature + 50)  # 90 values
    Bat_int = (int(((Millivolts+1) - 3000) / 50) + 20) % 40  # adding 1 to fix Python rounding  #40 values
    Speed_int = round((Speed+0.1) / 2.) # adding .1 to fix Python rounding  #42 values
    # GPS  # 2 values
    # Extra # 2 values

    Result1 = Temp_int*40 + Bat_int
    Result2 = Result1*42 + Speed_int
    Result3 = Result2*2 + GPSValid
    Result = Result3*2 + 1    # 1 = Standard Telemetry

    A = Result
    A_char_index = int(A / 34200)
    A_char = chr(  A_char_index + 65 )

    B = A - A_char_index*34200
    B_char_index = int( B / 1900 )
    B_char = chr(  B_char_index + 65 )

    C = B - B_char_index*1900
    C_char_index = int( C / 190 )
    C_char = chr(  C_char_index + 48 )

    D = C - C_char_index*190
    D_char_index = int( D / 19 )
    D_char = chr(  D_char_index + 48 )

    E = D - D_char_index*19
    E_char_index = E
    E_val = POWERS[E_char_index]


    out = [CALLSIGN[0] + \
        CALLSIGN[1] + \
        CALLSIGN[2] + \
        CALLSIGN[3] + \
        CALLSIGN[4] + \
        CALLSIGN[5], \
        A_char + B_char + C_char + D_char, \
        E_val]

    

    return out[0], out[1], out[2]







def CalcChecksum(inStr):
    checksum = 0
    for c in inStr: checksum = checksum ^ ord(c)
    return checksum

def VerifyChecksum(inStr):
    inArr = inStr.split('*')
    if len(inArr) != 2: return False
    if len(inArr[1]) != 2: return False  # must have 2 letters

    testChecksum = CalcChecksum(inArr[0][1:])
    actualChecksum = int(inArr[1], 16)
    return testChecksum == actualChecksum
    #return False



def ParseGPS_RMC(inStr):
    # $GPRMC,092750.000,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A*43    
    #    0       1      2     3     4     5      6   7    8     9   1011 12 
    
    if not VerifyChecksum(inStr):
        #eprint('RMC not valid')
        return None
    
    inArr = inStr.split(',')
    if len(inArr) < 13:
        #eprint('RMC too short')
        return None
    
    
    Valid = False
    if inArr[2] == 'A':
        Valid = True
        
    if not Valid:
        return [False]
        
        
    if len(inArr[1]) < 6:
        return None
    HH = int(inArr[1][0:2])
    MM = int(inArr[1][2:4])
    SSsss = float(inArr[1][4:])
    
    
    if len(inArr[3]) < 4:
        return None
    Lat = int(inArr[3][0:2]) + float(inArr[3][2:])/60.
    if inArr[4] == 'S':
        Lat = -Lat
    elif inArr[4] != 'N':
        # error
        return [False]
    
    if len(inArr[5]) < 5:
        return None
    Lon = int(inArr[5][0:3]) + float(inArr[5][3:])/60.
    if inArr[6] == 'W':
        Lon = -Lon
    elif inArr[6] != 'E':
        #error
        return [False]
    
    Knots = 0.0
    if len(inArr[7]) > 0:
        Knots = float(inArr[7])
    
    return [Valid, HH, MM, SSsss, Lat, Lon, Knots]


def ParseGPS_GGA(inStr):
    # $GPGGA,092751.000,5321.6802,N,00630.3371,W,1,8,1.03,61.7,M,55.3,M,,*75
    #    0       1         2      3   4        5 6 7  8    9  10  11 121314 
    
    if not VerifyChecksum(inStr):
        return None
    
    
    inArr = inStr.split(',')
    if len(inArr) < 15:
        return None
    
    
    Quality = int(inArr[6])
    if Quality == 0:
        return [0]
        
        
    if len(inArr[1]) < 6:
        return None
    HH = int(inArr[1][0:2])
    MM = int(inArr[1][2:4])
    SSsss = float(inArr[1][4:])
    
    
    if len(inArr[2]) < 4:
        return None
    Lat = int(inArr[2][0:2]) + float(inArr[2][2:])/60.
    if inArr[3] == 'S':
        Lat = -Lat
    
    if len(inArr[4]) < 5:
        return None
    Lon = int(inArr[4][0:3]) + float(inArr[4][3:])/60.
    if inArr[5] == 'W':
        Lon = -Lon
    
    # Alt MSL?
    Alt = 0
    if len(inArr[9]) > 0:
        Alt = float(inArr[9])
    
    
    return [Quality, HH, MM, SSsss, Lat, Lon, Alt]



def GetRMC(inStr, printFlag=False):
    if not 'RMC' in inStr: return
    
    global MY_RMC_VALID
    global MY_MM
    global MY_SS
    global MY_LAT
    global MY_LON
    global MY_SPEED
    global MY_GRID
    
    
    MY_RMC_VALID = False
    
    x = ParseGPS_RMC(inStr)    
    
    if x == None:
        if printFlag: eprint('RMC not complete message')
        
    elif len(x) == 1 and x[0] == False:
        if printFlag: eprint('RMC not valid')
        
    elif len(x) == 7:
        #[Valid, HH, MM, SSsss, Lat, Lon, Knots]
        if printFlag: 
            eprint('     RMC:')
            eprint('     Valid', x[0])
            eprint('     HH', x[1])
            eprint('     MM', x[2])
            eprint('     SS', x[3])
            eprint('     Lat', x[4])
            eprint('     Lon', x[5])
            eprint('     Knots', x[6])
            
        MY_RMC_VALID = x[0]
        MY_MM = x[2] % 10    #  0-9
        MY_SS = x[3]
        MY_LAT = x[4]
        MY_LON = x[5]
        MY_SPEED = x[6]
        
        MY_GRID = convertLatLonToGridSquare(MY_LAT, MY_LON)
        
    return




def ParseGPS_ZDA(inStr):

    if not VerifyChecksum(inStr):
        return None
    
    
    inArr = inStr.split(',')
    if len(inArr) < 7:
        return None
       
        
    if len(inArr[1]) < 6:
        return None
    HH = int(inArr[1][0:2])
    MM = int(inArr[1][2:4])
    SSsss = float(inArr[1][4:])
    
    return [HH, MM, SSsss]




def GetZDA(inStr, printFlag=False):
    if not 'ZDA' in inStr: return
    
    global MY_MM
    global MY_SS
    
    x = ParseGPS_ZDA(inStr)
    

    if x == None:
        if printFlag: eprint('ZDA not complete message')
    else:
        if printFlag: 
            eprint('              ZDA:')
            eprint('              MM', x[1])
            eprint('              SS', x[2])
        MY_MM = x[1] % 10  # 0-9
        MY_SS = x[2]
    return


def GetGGA(inStr, printFlag=False):
    if not 'GGA' in inStr: return

    global MY_GGA_QUALITY
    global MY_MM
    global MY_SS
    global MY_LAT
    global MY_LON
    global MY_ALT
    global MY_GRID
    global led
    
    led.toggle()

    MY_GGA_QUALITY = 0

    x = ParseGPS_GGA(inStr)

    if x == None:
        if printFlag: eprint('GGA not complete message')
        
    elif len(x) == 1 and x[0] == 0:
        if printFlag: eprint('GGA not good quality')
        
    elif len(x) == 7:
        if printFlag: 
            eprint('         GGA:')
            eprint('         Quality', x[0])
            eprint('         HH', x[1])
            eprint('         MM', x[2])
            eprint('         SS', x[3])
            eprint('         Lat', x[4])
            eprint('         Lon', x[5])
            eprint('         Alt', x[6])
        MY_GGA_QUALITY = x[0]
        MY_MM = x[2] % 10  # 0-9
        MY_SS = x[3]
        MY_LAT = x[4]
        MY_LON = x[5]
        MY_ALT = x[6]
        
        MY_GRID = convertLatLonToGridSquare(MY_LAT, MY_LON)
        
    return


def CreateNMEAString(inStr):
    checksumString = '{:02X}'.format(CalcChecksum(inStr))
    return '$' + inStr + '*' + checksumString + '\r\n'


  
def GetTemperature():
    
    Pico_Temp_C = int(27 - (((3.3/65535) * machine.ADC(4).read_u16()) - 0.706)/0.001721)
    Amb_Temp_C = -99
    Obj_Temp_C = -99
    
    try:
        Amb_Temp_C = sensor.read_ambient_temp()
        Obj_Temp_C = sensor.read_object_temp()
        eprint('Ambient {:.1f} C     Object {:.1f} C'.format(Amb_Temp_C, Obj_Temp_C))
    except:
        # an error happened, so use the Pico temperature instead
        eprint(' *** An error occured when reading the temperature sensor!')
        Amb_Temp_C = Pico_Temp_C
        Obj_Temp_C = Pico_Temp_C
    
    return Pico_Temp_C, Amb_Temp_C, Obj_Temp_C


def GetVoltage():
    return machine.ADC(29).read_u16() * 3.0 * 3.3 / 65535



def uart_read(uart):
    rxData = bytes()
    while uart.any() > 0:
        rxData += uart.read(1)
    return rxData


def uart_write(uart, outStr):
    uart.write(bytearray(outStr, 'utf-8'))



def RestartGPS(level):
    # 0 = hot
    # 1 = warm
    # 2 = cold
    # 3 = factory reset
    uart_write(uart1, CreateNMEAString('PCAS10,'+str(level)))

    
    

def GetGPSUpdate():
    eprint('GetGPSUpdate')
    global MY_RMC_VALID
    global MY_GGA_QUALITY
    global led
    #FeedWatchdog()
    led.value(0)  # LED OFF
    
    counter = 0
    deadline = time.ticks_add(time.ticks_ms(), 20 * 60 * 1000)  # 20 minutes from now

    accData = ''
    while True:
        counter += 1
        if counter >= 100:
            counter = 0
            #FeedWatchdog()
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                hardReset()
            
        incoming = uart_read(uart1)
        if len(incoming) > 0:
            try:
                decoded = incoming.decode('utf-8')
            except UnicodeError:
                decoded = ''
            if len(accData) and ('\r' in decoded or '\n' in decoded):
                eprint(accData)
                GetRMC(accData, True)
                GetGGA(accData, True)
                GetZDA(accData, True)
                #eprint(MY_RMC_VALID, MY_GGA_QUALITY)
                if isGPSDataValid():
                    #eprint('found valid, returning')
                    led.value(0)  # LED OFF
                    return
                accData = ''
            elif '$' in decoded:
                accData = decoded
            else:
                accData += decoded
                


def SetMyCallsign():
    global MY_CALLSIGN
    MY_CALLSIGN = properlyFormatCallsign(MY_CALLSIGN)
    if MY_CALLSIGN == None:
        return False
    return True

def SetChannelParameters():
    global MY_CHANNEL
    global MY_CHANNEL_TIME_MM
    global MY_CHANNEL_LANE
    global MY_SIGGENPARAMS
    MY_CHANNEL = int(MY_CHANNEL)
    if MY_CHANNEL < 0 or MY_CHANNEL > 599:
        return False
    
    MY_CHANNEL_TIME_MM = GetChannelTime(MY_CHANNEL)
    MY_CHANNEL_LANE = GetChannelLane(MY_CHANNEL)
    if MY_CHANNEL_TIME_MM < 0 or MY_CHANNEL_TIME_MM > 8 or (MY_CHANNEL_TIME_MM % 2) != 0:
        return False
    elif MY_CHANNEL_LANE < 1 or MY_CHANNEL_LANE > 4:
        return False


    MY_SIGGENPARAMS = SIGGENPARAMS[MY_BAND][MY_CHANNEL_LANE-1]
    if len(MY_SIGGENPARAMS) != 4:
        return False

    return True



def SetTXDeadlines(time_seconds_delay):
    global MY_TXTONE_DEADLINE
    global MY_TXWSPR_DEADLINE
    global MY_TXU4B_DEADLINE
    global MY_TXSLOT1_DEADLINE
    global MY_TXSLOT2_DEADLINE
    global MY_TXSLOT3_DEADLINE
    global MY_TXSLOT4_DEADLINE
    MY_TXTONE_DEADLINE = time.ticks_add(time.ticks_ms(), int(time_seconds_delay)*1000)
    MY_TXWSPR_DEADLINE = time.ticks_add(MY_TXTONE_DEADLINE, WARMUP_PERIOD_SEC*1000)
    MY_TXU4B_DEADLINE = time.ticks_add(MY_TXWSPR_DEADLINE, 2*60*1000)
    MY_TXSLOT1_DEADLINE = MY_TXU4B_DEADLINE
    MY_TXSLOT2_DEADLINE = time.ticks_add(MY_TXWSPR_DEADLINE, 4*60*1000)
    MY_TXSLOT3_DEADLINE = time.ticks_add(MY_TXWSPR_DEADLINE, 6*60*1000)
    MY_TXSLOT4_DEADLINE = time.ticks_add(MY_TXWSPR_DEADLINE, 8*60*1000)


def DelayAfterUpdate():
    global MY_CHANNEL_TIME_MM
    global WARMUP_PERIOD_SEC
    global MY_RMC_VALID
    global MY_GGA_QUALITY
    global MY_MM
    global MY_SS
    global MY_TXTONE_DEADLINE
    global IS_CONNECTED
    
    eprint('DelayAfterUpdate')
    eprint('     MY_CHANNEL_TIME_MM', MY_CHANNEL_TIME_MM)
    eprint('     WARMUP_PERIOD_SEC', WARMUP_PERIOD_SEC)
    eprint('     MY_MM', MY_MM)
    eprint('     MY_SS', MY_SS)
    eprint('     MY_RMC_VALID', MY_RMC_VALID)
    eprint('     MY_GGA_QUALITY', MY_GGA_QUALITY)

    if isGPSDataValid():
        
        time_seconds_delay = 600
    
        if isGeofenced(MY_GRID):
            time_seconds_delay = 599
            eprint('Geofenced in grid ', MY_GRID)
        else:
            time_seconds_to_start_tx = MY_CHANNEL_TIME_MM*60. - WARMUP_PERIOD_SEC
            if time_seconds_to_start_tx < 0:
                time_seconds_to_start_tx += 600
            time_seconds_right_now = MY_MM*60. + MY_SS
            time_seconds_delay = time_seconds_to_start_tx - time_seconds_right_now
            if time_seconds_delay < 0:
                time_seconds_delay += 600
            
            eprint('     time_seconds_delay', time_seconds_delay)
        
        
        if 2 < time_seconds_delay and time_seconds_delay < 20:
            SetTXDeadlines(time_seconds_delay)
            disableGPS()
            DelayMsAndFeed((time_seconds_delay-2) * 1000)
        elif 120 < time_seconds_delay and time_seconds_delay < 600:
            disableGPS()
            if IS_CONNECTED:
                DelayMsAndFeed((time_seconds_delay-60) * 1000)
            else:
                LightSleepMs((time_seconds_delay-60) * 1000)
                #eprint('returned from called to LightSleepMs()')
                restartUART()
            InvalidateGPS()
        else:
            InvalidateGPS()
    return


def PowerOnTransmitter():
    global WARMUP_PERIOD_SEC
    global MY_VOLTAGE
    global led
    eprint('PowerOnTransmitter', WARMUP_PERIOD_SEC)
    led.value(1)  # LED ON
    emitSymbol(1)
    DelayMsAndFeed((WARMUP_PERIOD_SEC-1) * 1000)
    MY_VOLTAGE = GetVoltage()
    stopSymbol()
    led.value(0)  # LED OFF
    

def MakeWSPRSymbols(formattedCallsignSr, grid4Str, powerDBmNum):
    C = encode(formattedCallsignSr, grid4Str, powerDBmNum)
    S = convolutionalCoding(C)
    D = interleave(S)
    WSPR_Symbols = mergeWithSync(D)
    
    for i,c in enumerate(WSPR_Symbols):
        WSPR_Symbols[i] = str(c)
        
    return WSPR_Symbols


def TxWSPR():
    global MY_CALLSIGN
    global MY_LAT
    global MY_LON
    global MY_POWER
    global MY_GRID
    global led
    
    led.value(0) # LED OFF
    
    
    eprint('TxWSPR()  2 min')
    eprint('     ['+MY_CALLSIGN+']')
    eprint('     Lat:', MY_LAT, ' Lon', MY_LON)
    eprint('     Grid:', MY_GRID)
    eprint('     Power:', MY_POWER)
    
        
    WSPR_Symbols = MakeWSPRSymbols(MY_CALLSIGN, MY_GRID[0:4], MY_POWER)
        
    wspr_test = ' '.join(WSPR_Symbols)
    eprint(wspr_test)
    TxWSPRSequence(WSPR_Symbols)
    
    led.value(0) # LED OFF
    

    

def TxU4B():
    global MY_ALT
    global MY_SPEED
    global MY_TEMPERATURE
    global MY_LAT
    global MY_LON
    global MY_CHANNEL
    global MY_GRID
    global led
    
    led.value(0) # LED OFF
    
    eprint('TxU4B() 2 min')
    eprint('     Alt:', MY_ALT)
    eprint('     Speed:', MY_SPEED)
    eprint('     Temp:', MY_TEMPERATURE)
    eprint('     Lat:', MY_LAT, ' Lon', MY_LON)
    eprint('     Volts:', MY_VOLTAGE)
    eprint('     MY_GRID[4:6]', MY_GRID[4:6])
    
    call6, grid4, ipowerdbm = EncodeU4B(MY_CHANNEL, MY_GRID[4:6], MY_ALT, MY_TEMPERATURE, MY_VOLTAGE*1000, MY_SPEED, True)
    
    eprint('     ' + call6 + ' ' + grid4 + ' ' + str(ipowerdbm))
    
        
    WSPR_Symbols = MakeWSPRSymbols(call6, grid4, ipowerdbm)
        
        
    wspr_test = ' '.join(WSPR_Symbols)
    eprint(wspr_test)
    TxWSPRSequence(WSPR_Symbols)
    
    led.value(0) # LED OFF
    
    

def DelayAfterTx():
    global IS_CONNECTED
    delay_minutes = 4
    if MY_ET_ENABLE: delay_minutes = 2
    eprint('DelayAfterTx() minutes:', delay_minutes)
    if IS_CONNECTED:
        DelayMsAndFeed(delay_minutes * 60 * 1000)
    else:
        LightSleepMs(delay_minutes * 60 * 1000)
        restartUART()


def TestForSi5351():
    PowerOsc.value(0) # ON
    time.sleep(0.2)
    devlist = i2c0.scan()
    PowerOsc.value(1) # OFF
    #eprint(devlist)
    for item in devlist:
        if item == 0x60: return True
    return False


def TestForMLX90614(x_i2c):
    addr = _I2C_Address
    devlist = x_i2c.scan()
    for item in devlist:
        if item == addr: return True
    return False


def TestBandString():
    global MY_BAND
    
    if not isinstance(MY_BAND, str):
        return False
    
    MY_BAND = MY_BAND.lower()
    
    if MY_BAND == '10m' or \
        MY_BAND == '12m' or \
        MY_BAND == '15m' or \
        MY_BAND == '17m' or \
        MY_BAND == '20m' or \
        MY_BAND == '22m' or \
        MY_BAND == '30m' or \
        MY_BAND == '40m' or \
        MY_BAND == '80m' or \
        MY_BAND == '160m':
            return True

    return False



def TxET(SlotNum):
    global MY_BIGCOUNTER
    
    led.value(0) # LED OFF
    
    #ValueArr = [123456789]
    encodedTemps = encodeBigNumTemperatures()
    
    if MY_BIGCOUNTER > 255 or MY_BIGCOUNTER < 0: MY_BIGCOUNTER = 0
    
    final_value = encodedTemps * 256 + MY_BIGCOUNTER
    
    ValueArr = [final_value]
    MY_BIGCOUNTER += 1
    
    
    HdrTelemetryType = 0 # always 0
    HdrRESERVED = 0  # always 0
    HdrType = 0  # probably 0
    HdrSlot = SlotNum
    
    encodedNumber = EncodeExtTelemValues(HdrTelemetryType, HdrRESERVED, HdrType, HdrSlot, ValueArr)
    
    call6, grid4, ipowerdbm = EncodeExtTelemetry(MY_CHANNEL, encodedNumber)

    eprint('   TxET()  ' + call6 + ' ' + grid4 + ' ' + str(ipowerdbm))
    
    WSPR_Symbols = MakeWSPRSymbols(call6, grid4, ipowerdbm)
        
        
    wspr_test = ' '.join(WSPR_Symbols)
    eprint(wspr_test)
    TxWSPRSequence(WSPR_Symbols)
    
    led.value(0) # LED OFF
    


def blinkForeverLEDError(count):
    led.value(0)
    while True:
        #FeedWatchdog()
        led.value(1)
        time.sleep(0.6)
        led.value(0)
        time.sleep(0.3)
        for i in range(count):
            #FeedWatchdog()
            led.value(1)
            time.sleep(0.2)
            led.value(0)
            time.sleep(0.3)
        time.sleep(1.0)




# actual running


machine.freq(48000000)   # CPU clock speed = 48 MHz
eprint('CPU clock speed = ', machine.freq())
#FeedWatchdog()


led = Pin("LED", Pin.OUT)
led.value(1)  # LED ON


Pico_Vsys = Pin(29, Pin.IN)  # Voltage Vsys input

Pico4 = Pin(2, Pin.OUT)  # GPS, low = ON
Pico34 = Pin(28, Pin.OUT) # Osc, low = ON
Pico9 = Pin(6, Pin.OUT)  # GPS NRESET, low = reset
Pico5 = Pin(3, Pin.OUT)  # GPS VBAT, high = on

PowerGPS = Pico4
PowerOsc = Pico34
NRESETGPS = Pico9
VBATGPS = Pico5

uart1 = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9))

i2c0= I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)
i2c1 = I2C(_I2C_Channel, scl=Pin(_I2C_GPIO_SCL_Pin), sda=Pin(_I2C_GPIO_SDA_Pin), freq=100000)
time.sleep_ms(200)

if not TestForMLX90614(i2c1):
    eprint('MLX90614 not found')
    #blinkForeverLEDError(6)
    sensor = None
else:
    sensor = MLX90614(i2c1)
    time.sleep(0.2)
    GetTemperature()


# Set GPS to reset state
NRESETGPS.value(0)  # 0 = reset
VBATGPS.value(1)   # 1 = enable battery backup power

# sleep delay
time.sleep(0.25)

# Turn off everything
PowerGPS.value(1)  # 1=OFF
PowerOsc.value(1)  # 1=OFF

# sleep delay
time.sleep(0.5)
#FeedWatchdog()





if not TestForSi5351():
    eprint('SI5351 oscillator not powering up')
    blinkForeverLEDError(2)
elif not SetMyCallsign():
    eprint('Invalid callsign')
    blinkForeverLEDError(3)
elif not TestBandString():
    eprint('Invalid band')
    blinkForeverLEDError(5)
elif not SetChannelParameters():
    eprint('Invalid channel')
    blinkForeverLEDError(4)
else:
    while True:                   # loop forever
        enableGPS()               # GPS chip on
        GetGPSUpdate()            # Get time, location, speed
        DelayAfterUpdate()        # wait until TX time
        if isGPSDataValid():
            
            # Handle temperature
            PicoTemp, AmbientTemp, ObjectTemp = GetTemperature()  # get temperature during cold
            MY_TEMPERATURE = ObjectTemp   # choose the reported temperature source
            
            disableGPS()          # GPS chip off
            enableOsc()           # TXCO and SI5351 on
            DelayMsDeadlineAndFeed(MY_TXTONE_DEADLINE)
            PowerOnTransmitter()  # stable tone signal, ~30 sec
            DelayMsDeadlineAndFeed(MY_TXWSPR_DEADLINE)
            TxWSPR()              # CALLSIGN GRID POWER  2 min
            DelayMsDeadlineAndFeed(MY_TXU4B_DEADLINE)
            TxU4B()               # LOCATION ALT TEMP VOLTAGE SPEED 2 min
            if MY_ET_ENABLE:
                DelayMsDeadlineAndFeed(MY_TXSLOT2_DEADLINE)
                TxET(MY_ET_SLOT)
            disableOsc()          # TXCO and SI5351 off
            InvalidateGPS()       # discard GPS data
            DelayAfterTx()        # long wait after the final TX
        