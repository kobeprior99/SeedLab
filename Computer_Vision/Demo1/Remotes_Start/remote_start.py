'''
*******************************************************************
* File Name         : remote_start.py

* Description       : Send instructions to Arduino via I2C
*
* Supplementary File(s): using_remote_start.ino shows example excecution  
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 
* 02/22/2025    Kobe Prior Created File
******************************************************************
Hardware Setup: 
-connect gnd to gnd, sda to sda, and scl to scl from pi to arduino.
-connect pin 5 on Pi to pin A5 on Arduino (SCL)
-connect pin 6 on Pi to GND on Arduino

Example Execution: 
- run the script using: python remote_start.py after navigating to the correct directory
- Enter a distance and angle when prompted
- The distance and angle will be sent to the arduino via I2C

'''
from smbus2 import SMBus

import struct
#I2c to communicate with the arduino
ARD_ADDR = 8 #set arduino address
i2c_arduino = SMBus(1)#initialize i2c bus to bus 1

# function to send instructions to the arduino
def send_instructions(angle, distance):
    """
    Sends instructions to an Arduino via I2C communication.

    Parameters:
    angle (float): The angle value to send.
    distance (float): The distance value to send.

    Raises:
    IOError: If the I2C write operation fails.
   git  """
    #handle exception if i2c write fails
    try:
        #future proof for sending floats
        # instruction_array [good_angle, angle, good_distance, distance, good_arrow, arrow
        # 1.0 is valid, 0.0 is invalid
        instruction_array = [1.0, angle, 1.0, distance, 0.0, 1.1]
        byte_array = bytearray()
        #floats have to be sent a special way and decoded in a special way
        for instruction in instruction_array:
            byte_array.extend(struct.pack("f", instruction))

        #parameters are address of arduino, register to write to, and data to write
        i2c_arduino.write_i2c_block_data(ARD_ADDR, 0, list(byte_array))

    except IOError:
        print("Could not write data to the Arduino.")


#get distance and angle from user
angle = float(input("enter an angle in degrees: "))
distance =float(input("enter a distance in inches: "))
#send the instructions
send_instructions(angle, distance)

#imagine now we are sending these instructions to the arduino during every frame 
#the arduino would be able to tell distance to marker, angle of marker, and color of arrow, arrow next to marker, and if these parameters were good or not