"""Benewake TFmini-S / TF-Luna UART protocol, without ROS or pyserial.

Both sensors stream the standard 9-byte frame (factory default, 115200 8N1,
100 Hz, distance in cm):

    0x59 0x59 Dist_L Dist_H Strength_L Strength_H Temp_L Temp_H Checksum

Checksum is the low byte of the sum of the first eight bytes, and the chip
temperature is Temp / 8 - 256 degC. FrameParser resynchronises on the
header after garbage or a bad checksum, so it can be fed any chunking of
the byte stream.

interpret() turns a frame into a sensor_msgs/Range value following REP 117:
a finite range in metres when valid, +inf for no return (weak signal or
beyond max_range), -inf for a target closer than min_range, and NaN for an
erroneous reading (signal or ambient-light saturation). Abnormal values
from the product manuals (SJ-PM-TFmini-S A01 table 8, SJ-PM-TF-Luna A05
section 6.4/6.8):

    TFmini-S  Strength < 100         -> Dist 65535 (-1)
              Strength == 65535      -> Dist 65534 (-2)  signal saturation
              ambient light saturated-> Dist 65532 (-4)
    TF-Luna   Amp < 100              -> Dist 0 (dummy_dist)
              Amp == 65535           -> overexposure
              Amp > 32768            -> ambient light overexposure (sun)
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

HEADER = b'\x59\x59'
FRAME_LENGTH = 9

# Signal strength limits shared by both sensors (manuals: unreliable below
# 100, 65535 marks saturation/overexposure).
MIN_STRENGTH = 100
SATURATED_STRENGTH = 0xFFFF

# TFmini-S distance sentinels (two's complement -1, -2, -4 in cm).
DIST_WEAK = 0xFFFF
DIST_SATURATED = 0xFFFE
DIST_AMBIENT = 0xFFFC

# Reading status names (also used in log messages)
OK = 'ok'
WEAK_SIGNAL = 'weak_signal'
SATURATED = 'saturated'
AMBIENT_LIGHT = 'ambient_light'
TOO_CLOSE = 'too_close'
TOO_FAR = 'too_far'


@dataclass(frozen=True)
class ModelSpec:
    """Datasheet limits of one sensor model (90% reflectivity target)."""
    min_range_m: float
    max_range_m: float
    field_of_view_rad: float
    # Strength above which the sensor reports ambient-light overexposure,
    # or None when the model does not define one.
    ambient_strength: Optional[int] = None


MODELS = {
    # TFmini-S: 0.1-12 m (0-10 cm blind zone), FOV 2 deg, 5 V supply,
    # 3.3 V LVTTL UART.
    'tfmini_s': ModelSpec(0.1, 12.0, math.radians(2.0)),
    # TF-Luna: 0.2-8 m, FOV 2 deg, 3.7-5.2 V supply, 3.3 V LVTTL UART.
    'tf_luna': ModelSpec(0.2, 8.0, math.radians(2.0), ambient_strength=32768),
}


@dataclass(frozen=True)
class Frame:
    distance_cm: int
    strength: int
    temperature_c: float


def checksum(data: bytes) -> int:
    """Low byte of the sum of the first eight frame bytes."""
    return sum(data[:FRAME_LENGTH - 1]) & 0xFF


def encode_frame(distance_cm: int, strength: int, temperature_c: float = 25.0) -> bytes:
    """Build a valid 9-byte frame (for tests and the simulated serial feed)."""
    raw_temp = int(round((temperature_c + 256.0) * 8.0)) & 0xFFFF
    body = bytes([
        0x59, 0x59,
        distance_cm & 0xFF, (distance_cm >> 8) & 0xFF,
        strength & 0xFF, (strength >> 8) & 0xFF,
        raw_temp & 0xFF, (raw_temp >> 8) & 0xFF,
    ])
    return body + bytes([checksum(body)])


def decode_frame(data: bytes) -> Optional[Frame]:
    """Decode one 9-byte frame, or None if the header or checksum is wrong."""
    if len(data) != FRAME_LENGTH or data[:2] != HEADER:
        return None
    if checksum(data) != data[8]:
        return None
    raw_temp = data[6] | (data[7] << 8)
    return Frame(
        distance_cm=data[2] | (data[3] << 8),
        strength=data[4] | (data[5] << 8),
        temperature_c=raw_temp / 8.0 - 256.0,
    )


class FrameParser:
    """Incremental decoder for the 9-byte frame stream.

    feed() accepts arbitrary chunks (split frames, leading garbage, line
    noise) and returns the frames completed so far. A header whose
    checksum fails is skipped one byte at a time so that a real frame
    starting inside it (e.g. a distance byte of 0x59) is still found.
    """

    def __init__(self):
        self._buffer = bytearray()
        self.frames = 0
        self.checksum_errors = 0
        self.discarded_bytes = 0

    def reset(self):
        """Drop any partial frame, e.g. after reopening the port."""
        self._buffer.clear()

    def feed(self, data: bytes) -> List[Frame]:
        self._buffer.extend(data)
        frames = []
        buffer = self._buffer
        while True:
            start = buffer.find(HEADER)
            if start < 0:
                # Keep a trailing 0x59: it may be the first header byte.
                keep = 1 if buffer[-1:] == HEADER[:1] else 0
                self.discarded_bytes += len(buffer) - keep
                del buffer[:len(buffer) - keep]
                break
            if start:
                self.discarded_bytes += start
                del buffer[:start]
            if len(buffer) < FRAME_LENGTH:
                break
            frame = decode_frame(bytes(buffer[:FRAME_LENGTH]))
            if frame is None:
                self.checksum_errors += 1
                self.discarded_bytes += 1
                del buffer[:1]
                continue
            del buffer[:FRAME_LENGTH]
            self.frames += 1
            frames.append(frame)
        return frames


def interpret(frame: Frame, min_range_m: float, max_range_m: float,
              ambient_strength: Optional[int] = None) -> Tuple[float, str]:
    """REP 117 range value in metres and a status name for one frame."""
    if (frame.strength == SATURATED_STRENGTH or
            frame.distance_cm == DIST_SATURATED):
        return math.nan, SATURATED
    if (frame.distance_cm == DIST_AMBIENT or
            (ambient_strength is not None and frame.strength > ambient_strength)):
        return math.nan, AMBIENT_LIGHT
    if frame.strength < MIN_STRENGTH or frame.distance_cm == DIST_WEAK:
        # No usable return: nothing within range (REP 117: +inf).
        return math.inf, WEAK_SIGNAL
    distance_m = frame.distance_cm / 100.0
    if distance_m < min_range_m:
        return -math.inf, TOO_CLOSE
    if distance_m > max_range_m:
        return math.inf, TOO_FAR
    return distance_m, OK
