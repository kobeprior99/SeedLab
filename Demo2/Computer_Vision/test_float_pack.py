# Author: Kobe Prior
# Date: March 24, 2025
# Description: Test converting float into list of bytes for I2C send
import struct
convert_to_bytes = 3.14159
#convert float into bytes
float_bytes = struct.pack('f', convert_to_bytes)
print(list(float_bytes))

