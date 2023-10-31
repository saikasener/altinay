#!/usr/bin/env python3

# ------------------------------------------------------
# ----------------------- Module -----------------------
# ------------------------------------------------------
from rpi_ws281x import *
import RPi.GPIO as GPIO
import time
import rospy
from moobot_msgs.msg import moobot_status, lift_status, moobot_scanner, moobot_sensor_status

# ------------------------------------------------------
# ---------------------- Function ----------------------
# ------------------------------------------------------
# LED strip configuration:
LED_COUNT      = 677      # Number of LED pixels. 276
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255    # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ,LED_DMA,LED_INVERT,LED_BRIGHTNESS,LED_CHANNEL)
strip.begin()

# -----------------------------------------------------------------
# --------------------- Total Led Count = 677 ---------------------
# LeftSideLedNumber = [1- 234]      -> 235 LED Start Point loop -> 0 234
# BackLedNumber = [235 - 340]       -> 107 LED             loop -> 234 340
# RightSideLedNumber = [341 - 572]  -> 233 LED             loop -> 340 572
# FrontLedNumber = [573 - 677]      -> 106 LED End Point   loop -> 572 677
# -----------------------------------------------------------------
def main():
    SetStartAgv()
    while True:        
#        for x in range(0,LED_COUNT):
#           strip.setPixelColor(x,Color(255,0,0))
#        strip.show()
#        time.sleep(.5)
        SetClear()
#        time.sleep(.5)
        for x in range(0,LED_COUNT):
            strip.setPixelColor(x,Color(255,0,0))
            strip.setPixelColor(677-x,Color(0,0,255))
            strip.show()
#        time.sleep(.5)
#        for x in range(0,LED_COUNT):
#            strip.setPixelColor(x,Color(0,255,0))
#        strip.show()
#        time.sleep(.2)
#        for x in range(0,LED_COUNT):
#            strip.setPixelColor(x,Color(0,0,255))
#        strip.show()
#        time.sleep(.5)
## ------------------------------------------------------
def SetClear():
    for x in range(0,LED_COUNT):
        strip.setPixelColor(x,Color(0,0,0))
    strip.show()
# ------------------------------------------------------
def SetStartAgv():
#    SetClear() # Clear All led
# kodumun hatası
    for i in range(0,50):
        for x in range(0,LED_COUNT):
            strip.setPixelColor(x,Color(255,0,(250-i*4)))
        strip.show()
        time.sleep(.001)
    time.sleep(2)
    SetClear()
    time.sleep(1)
# ------------------------------------------------------
        
        
# ------------------------------------------------------
# ------------------------------------------------------
if __name__ == "__main__":
    main()
else:
    print("Error!")
        
        
