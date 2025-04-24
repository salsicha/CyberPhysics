



Compile:
dtc -I dts -O dtb -o pwm-pi5.dtbo pwm-pi5-overlay.dts

Install:
sudo cp pwm-pi5.dtbo /boot/firmware/overlays/

Enable:
sudo python3 pwm/enable.py

Test:
sudo python3 test.py


Final test:
connect LEDs to pins 12, 13, 18, 19 and test



https://github.com/danjperron/Pi5PWM_HARDWARE

https://gist.github.com/Gadgetoid/b92ad3db06ff8c264eef2abf0e09d569

https://forums.raspberrypi.com/viewtopic.php?t=363122
