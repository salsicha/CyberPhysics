# toolkit.pyx

cimport numpy as np
import numpy as np
import struct
import sys
import math
import io
import RPi.GPIO as GPIO

def float_to_bytes(f:float) -> bytes:
    b = struct.pack("f", f)
    return b

def bytes_to_float(bs:bytes) -> float:
    return struct.unpack("f", bs)[0]

cdef class ControlCommand:

    cdef public int frame
    cdef public double throttle
    cdef public double roll
    cdef public double pitch
    cdef public double yaw
    cdef public double checksum

    def __init__(self):
        self.frame = 0
        self.throttle = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.checksum = 0.0

    def calculate_checksum(self) -> double:
        cdef double ToReturn = 0.0
        ToReturn += float(self.frame)
        ToReturn += self.throttle
        ToReturn += self.roll
        ToReturn += self.pitch
        ToReturn += self.yaw
        return ToReturn

    def checksum_correct(self, tolerance:double = 0.00001) -> bool:
        cdef double diff = abs(self.checksum - self.calculate_checksum())
        return diff < tolerance

    def encode(self) -> bytes:
        cdef bytearray ToReturn = bytearray()
        
        if sys.platform == "rp2":
            frame_bytes = self.frame.to_bytes(4, 0)
        else:
            frame_bytes = self.frame.to_bytes(4, byteorder='big')
        for b in frame_bytes:
            ToReturn.append(b)

        ToReturn.extend(float_to_bytes(self.throttle))
        ToReturn.extend(float_to_bytes(self.roll))
        ToReturn.extend(float_to_bytes(self.pitch))
        ToReturn.extend(float_to_bytes(self.yaw))

        ToReturn.extend(float_to_bytes(self.calculate_checksum()))

        return bytes(ToReturn)

    def decode(self, bs:bytes) -> None:
        if sys.platform == "rp2":
            self.frame = int.from_bytes(bs[0:4], 0)
        else:
            self.frame = int.from_bytes(bs[:4], byteorder='big')

        self.throttle = bytes_to_float(bs[4:8])
        self.roll = bytes_to_float(bs[8:12])
        self.pitch = bytes_to_float(bs[12:16])
        self.yaw = bytes_to_float(bs[16:20])

        self.checksum = bytes_to_float(bs[20:24])


cdef class PIDCommand:

    cdef public int axis
    cdef public double kp
    cdef public double ki
    cdef public double kd

    def __init__(self):
        self.axis = 0
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0

    def encode(self) -> bytes:
        cdef bytearray ToReturn = bytearray()
        ToReturn.append(self.axis)
        ToReturn.extend(float_to_bytes(self.kp))
        ToReturn.extend(float_to_bytes(self.ki))
        ToReturn.extend(float_to_bytes(self.kd))
        return bytes(ToReturn)
    
    def decode(self, bs:bytes) -> None:
        self.axis = bs[0]
        self.kp = bytes_to_float(bs[1:5])
        self.ki = bytes_to_float(bs[5:9])
        self.kd = bytes_to_float(bs[9:13])


cdef class TelemetryFrame:

    cdef public int time
    cdef public double accel_x
    cdef public double accel_y
    cdef public double accel_z
    cdef public double gyro_x
    cdef public double gyro_y
    cdef public double gyro_z
    cdef public double pitch_angle
    cdef public double roll_angle

    def __init__(self):
        self.time = 0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.pitch_angle = 0.0
        self.roll_angle = 0.0

    def encode(self) -> bytes:
        cdef bytearray ToReturn = bytearray()

        if sys.platform == "rp2":
            for b in self.time.to_bytes(4, 0):
                ToReturn.append(b)
        else:
            for b in self.time.to_bytes(4, byteorder='big'):
                ToReturn.append(b)

        ToReturn.extend(float_to_bytes(self.accel_x))
        ToReturn.extend(float_to_bytes(self.accel_y))
        ToReturn.extend(float_to_bytes(self.accel_z))
        ToReturn.extend(float_to_bytes(self.gyro_x))
        ToReturn.extend(float_to_bytes(self.gyro_y))
        ToReturn.extend(float_to_bytes(self.gyro_z))
        ToReturn.extend(float_to_bytes(self.pitch_angle))
        ToReturn.extend(float_to_bytes(self.roll_angle))

        return bytes(ToReturn)
    
    def decode(self, data:bytes) -> None:
        if len(data) != len(TelemetryFrame().encode()):
            raise Exception("Unable to decode: the input data was not correct.")
        

        if sys.platform == "rp2":
            self.time = int.from_bytes(data[0:4], 0)
        else:
            self.time = int.from_bytes(data[0:4], byteorder='big')

        self.accel_x = bytes_to_float(data[4:8])
        self.accel_y = bytes_to_float(data[8:12])
        self.accel_z = bytes_to_float(data[12:16])
        self.gyro_x = bytes_to_float(data[16:20])
        self.gyro_y = bytes_to_float(data[20:24])
        self.gyro_z = bytes_to_float(data[24:28])
        self.pitch_angle = bytes_to_float(data[28:32])
        self.roll_angle = bytes_to_float(data[32:36])
        

    def save(self, opened_file:io.BufferedWriter = None) -> None:
        """Appends the frame, in bytes, to the 'telemetry' file in the root directory."""
        
        if opened_file == None:
            with open("telemetry", "ab") as f:
                f.write(self.encode())
        else:
            opened_file.write(self.encode())

    
    @staticmethod
    cdef encode_frames(frames:list["TelemetryFrame"]) -> bytes:
        cdef bytearray ToReturn = bytearray()
        for frame in frames:
            for b in frame.encode():
                ToReturn.append(b)
        return bytes(ToReturn)
    
    @staticmethod
    cdef decode_frames(data:bytes) -> list["TelemetryFrame"]:

        cdef int frame_length = len(TelemetryFrame().encode())
        cdef int begin = 0
        cdef int end = frame_length

        cdef list[TelemetryFrame] ToReturn = []
        while end <= len(data):
            tf = TelemetryFrame()
            tf.decode(data[begin:end])
            ToReturn.append(tf)
            begin += frame_length
            end += frame_length

        return ToReturn




    def add_float_bytes(self, ba:bytearray, f:float) -> None:
        for b in struct.pack("f", f):
            ba.append(b)


cdef class NonlinearTransformer:
    """Converts a linear input to a nonlinear output (dampening) using tanh and a dead zone."""
    

    cdef public double nonlinearity_strength
    cdef public double dead_zone_percent

    cdef public double multiplier

    def __init__(self, nonlinearity_strength:double = 2.0, dead_zone_percent:double = 0.0) -> None:
        self.nonlinearity_strength = nonlinearity_strength
        self.dead_zone_percent = dead_zone_percent
        self.multiplier = nonlinearity_strength

    def y(self, x:float) -> float:
        return math.tanh(self.multiplier * (x - 1)) + 1

    cdef double _transform(self, percent:float) -> double:

        # account for dead zone
        cdef double x = (percent - self.dead_zone_percent) / (1.0 - self.dead_zone_percent)
        x = max(x, 0.0)
        x = min(x, 1.0)

        # determine the range we have to work with (minimum is tanh intersect at 0.0 x)
        cdef double min_y = self.y(0)
        cdef double max_y = 1.0
    
        # calculate and scale to within the min and max range
        cdef double ToReturn = self.y(x)
        ToReturn = (ToReturn - min_y) / (max_y - min_y)
        return ToReturn
    
    def transform(self, percent:float) -> float:
        """Convert linear input to nonlinear output."""
        if percent >= 0:
            return self._transform(percent)
        else:
            return (self._transform(abs(percent)) * -1)


cpdef log(msg:str) -> None:
    cdef str fp = "/logs"
    with open(fp, "a") as f:
        f.write(msg + "\n\n")