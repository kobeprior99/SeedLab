from smbus2 import SMBus
from time import sleep
ARD_ADDR = 8

i2c = SMBus(1)
while(True):
    offset = int(input("Enter an offset (7 to quit): "))
    if(offset == 7):
        break
    string = input("enter a string of 32 characters or less: ")
    command = [ord(characters) for characters in string]
    try:
        i2c.write_i2c_block_data(ARD_ADDR, offset, command)
    except IOError:
        print("Could not write data to the Arduino.")
    sleep(.1)
    reply = i2c.read_byte_data(ARD_ADDR, offset)
    print("recieved from Arduino:"+str(reply))
