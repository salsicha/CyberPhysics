# ibus.pyx
import serial
import time

cdef class IBus:
    cdef public int uart_port
    cdef public int baud
    cdef object ser
    cdef public int num_channels
    cdef list ch
    
    cpdef __init__(self, int uart_port, int baud=115200, int num_channels=6):
        self.uart_port = uart_port
        self.baud = baud
        self.ser = serial.Serial(self.uart_port, self.baud)
        self.num_channels = num_channels
        # Initialize channel values to 0
        self.ch = [0] * (num_channels + 1)

    cpdef list read(self):
        cdef int z
        for z in range(10):
            buffer = bytearray(31)
            char = self.ser.read(1)
            # Check for 0x20
            if char == b'\x20':
                # Read the rest of the string into buffer
                self.ser.readinto(buffer)
                checksum = 0xffdf  # 0xffff - 0x20
                # Check checksum
                cdef int i
                for i in range(29):
                    checksum -= buffer[i]
                if checksum == (buffer[30] << 8) | buffer[29]:
                    self.ch[0] = 1  # status 1 = success
                    for i in range(1, self.num_channels + 1):
                        self.ch[i] = (buffer[(i * 2) - 1] + (buffer[i * 2] << 8))
                    return self.ch
                else:
                    # Checksum error
                    self.ch[0] = -2
            else:
                self.ch[0] = -1
                
        # Reach here then timed out
        self.ch[0] = -1
        return self.ch

    @staticmethod
    cpdef normalize(int value, str type="default"):
        if type == "dial":
            return ((value - 1000) / 10)
        else:
            return ((value - 1500) / 5)


# Example usage
cdef main():
    ibus = IBus(0)  # Replace with your UART port
    while True:
        data = ibus.read()
        print(data)
        time.sleep(0.1)

if __name__ == "__main__":
    main()