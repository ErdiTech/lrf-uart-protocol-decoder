"""Laser rangefinder / single-point LiDAR UART frame decoder with protocol auto-detection.

>>> from lrf_decoder import detect, decode, parse_hex
>>> data = parse_hex("59 59 79 00 00 02 F0 08 25")
>>> detect(data)[0]["family"]
'benewake-9'
>>> round(decode(data, "benewake-tfmini-s")[0][0]["distance_m"], 3)
1.21
"""
from .core import BAUD_RATES, COMMANDS, PROFILES, Decoder, crc16ccitt, decode, detect, parse_hex

__version__ = "1.0.0"
__all__ = ["PROFILES", "COMMANDS", "BAUD_RATES", "Decoder", "decode", "detect", "parse_hex", "crc16ccitt", "__version__"]
