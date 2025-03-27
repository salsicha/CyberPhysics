

# Quadcopter flight controller.



## TODO:
update dockerfile to install aerostack2
get aerostack2 working in sim from examples
get aerostack2 flight controller running
keyboard publisher
get genesis sim version working 
get sensors working
etc...



Deprecated:
Original source code here:
https://github.com/timhanewich/scout



## Raspberry Pi 5 fixes:

```bash
sudo apt remove python3-rpi.gpio
sudo apt install python3-rpi-lgpio
```

I2C:
https://ozzmaker.com/i2c/
sudo raspi-config
I2C -> select -> enable

Set up venv:
sudo apt-get install python3-venv
cd ~
python -m venv venv
source ~/venv/bin/activate

Raspberry Pi 5 config:
Need to add 'dtoverlay=pwm-2chan' to /boot/firmware/config.txt and reboot




GPS/IMU/LTE
https://www.amazon.com/OZZMAKER-SARA-R5-LTE-M-GPS-10DOF/dp/B0BVLM75JZ



RPi5
https://www.amazon.com/gp/f.html?C=2C6047Q0RW2O1&K=31DQ08M34E2JY&M=urn:rtn:msg:202502031957441f52acd2c74a49afaa82aac72f70p0na&R=3CDWPGQW5D9A8&T=C&U=%2Fdp%2FB0CRSNCJ6Y%3Fref_%3Dpe_123509780_1038749300_fed_asin_title&H=EVPSXCLEGQ56UXWECVGTD9LHPCKA&ref_=pe_123509780_1038749300_fed_asin_title



Oak-1
https://www.amazon.com/dp/B0BR7WS519?ref_=pe_125775000_1044873430_fed_asin_title



Lidar range finder
https://www.amazon.com/Single-Point-Compatible-Rasppbarry-Communication-Interface/dp/B088NVX2L7/ref=asc_df_B088NVX2L7?mcid=d5da41b6f4e1368cb2c9c6636554cf64&hvocijid=3444563270662111785-B088NVX2L7-&hvexpln=73&tag=hyprod-20&linkCode=df0&hvadid=721245378154&hvpos=&hvnetw=g&hvrand=3444563270662111785&hvpone=&hvptwo=&hvqmt=&hvdev=m&hvdvcmdl=&hvlocint=&hvlocphy=9031945&hvtargid=pla-2281435178058&psc=1



ESC
https://www.amazon.com/RC-Brushless-Electric-Controller-bullet/dp/B071GRSFBD



Motors
https://www.amazon.com/gp/product/B07Y9JK2MW



Frame
https://dl.djicdn.com/downloads/flamewheel/en/F450_User_Manual_v2.2_en.pdf



Battery
3,300 mAh 4S lithium polymer (LiPo)



Propellers
https://www.amazon.com/gp/product/B07PVFS5YH



Wifi
https://github.com/DroneBridge/DroneBridge



Front camera
https://www.raspberrypi.com/products/ai-camera/
