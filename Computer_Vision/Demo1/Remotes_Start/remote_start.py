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
-connect pin 3 on Pi to pin A4 on Arduino (SDA)
-connect pin 5 on Pi to pin A5 on Arduino (SCL)
-connect pin 6 on Pi to GND on Arduino

Example Execution: 
- run the script using: python remote_start.py after navigating to the correct directory
- Enter a distance and angle when prompted
- The distance and angle will be sent to the arduino via I2C

'''
from smbus2 import SMBus

#I2c to communicate with the arduino
ARD_ADDR = 8 #set arduino address
i2c_arduino = SMBus(1)#initialize i2c bus to bus 1

# function to send instructions to the arduino
def send_instructions(distance, angle):
    """
    Sends instructions to an Arduino via I2C communication.

    Parameters:
    distance (int): The distance value to send.
    angle (int): The angle value to send.

    Raises:
    IOError: If the I2C write operation fails.
    """
    #handle exception if i2c write fails
    try:
        instruction_array = [distance, angle]
        #parameters are address of arduino, register to write to, and data to write
        i2c_arduino.write_i2c_block_data(ARD_ADDR, 0, instruction_array)
    except IOError:
        print("Could not write data to the Arduino.")


#get distance and angle from user
distance =int(input("enter a distance in inches: "))
angle = int(input("enter an angle in degrees: "))
#send the instructions
send_instructions(distance, angle)

