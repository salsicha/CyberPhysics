#!/usr/bin/python3
import os
import time

class pi5RC:

    def __init__(self,Pin):
        pins = [ 12,13,14,15,18,19]
        afunc= [ 'a0','a0','a0', 'a0', 'a3','a3'];
        self.pwmx = [ 0,1,2,3,2,3]
        self.enableFlag=False
        if  Pin in pins:
            self.pin=Pin
            self.pinIdx = pins.index(Pin)
            # let's set pin ctrl
            os.system("/usr/bin/pinctrl set {} {}".format(self.pin,afunc[self.pinIdx]))
            # let export pin
            os.system("echo {} > /sys/class/pwm/pwmchip0/export".format(self.pwmx[self.pinIdx]))
            # CLOCK AT 1gHZ  let put period to 20ms
            time.sleep(0.2)
            os.system("echo 20000000 > /sys/class/pwm/pwmchip0/pwm{}/period".format(self.pwmx[self.pinIdx]))
            time.sleep(0.1)
            self.enable(False)
        else:
            self.pin=None
            print("Error Invalid Pin")

    def enable(self,flag):
            self.enableFlag=flag
            os.system("echo {} > /sys/class/pwm/pwmchip0/pwm{}/enable".format(
                      int(self.enableFlag),self.pwmx[self.pinIdx]))


    def __del__(self):
       if self.pin is not None:
           #ok take PWM out
            os.system("echo {} > /sys/class/pwm/pwmchip0/unexport".format(self.pwmx[self.pinIdx]))
           #disable PWM Pin
            os.system("/usr/bin/pinctrl set {} no".format(self.pin))



    def set(self, onTime_us):
        if not self.enableFlag:
            self.enable(True)
        self.onTime_ns=onTime_us*1000
        os.system("echo {} > /sys/class/pwm/pwmchip0/pwm{}/duty_cycle".format(self.onTime_ns,self.pwmx[self.pinIdx]))
