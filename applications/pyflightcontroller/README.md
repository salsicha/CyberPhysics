
Python based quadcopter flight controller.

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



