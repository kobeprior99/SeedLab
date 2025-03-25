import struct
convert_to_bytes = 3.14159
#convert float into bytes
float_bytes = struct.pack('f', convert_to_bytes)
print(list(float_bytes))

