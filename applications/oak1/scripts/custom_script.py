''' 
This is an example script that gets and prints the local IP of the device
'''

import socket
import fcntl
import struct
import time
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        -1071617759,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15].encode())
    )[20:24])

while True:
    ip = get_ip_address('re0')  # '192.168.0.110'
    node.warn(f'IP of the device: {ip}')
    node.io['end'].send(Buffer(32))
    time.sleep(0.5)
