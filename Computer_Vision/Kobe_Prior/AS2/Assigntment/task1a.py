'''
*******************************************************************
* File Name         : task1a.py
* Description       : Use I2C to send a string from the Pi to the Arduino, 
* and the arduino displays the string characters, followed by the ASCII codes
*                    
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 01/26/2025	Kobe Prior	Created File
*
* Help: https://smbus2.readthedocs.io/en/latest/ 
******************************************************************
Hardware Setup: Power on the Pi -> either connect peripherials or connect via PiConnect or other VNC service 
-connect gnd to gnd, sda to sda, and scl to scl from pi to arduiono.
-connect pin 3 on Pi to pin A4 on Arduino (SDA)
-connect pin 5 on Pi to pin A5 on Arduino (SCL)
-connect pin 4 on Pi to GND on Arduino
Example Excecution: open arudino ide and upload the task1a.ino file to the arduino, then open serial monitor
->open terminal -> navigate to the directory where the task1a.py file is located using cd command ->
run the python file using 'python task1a.py' command

'''
# Import the necessary libraries
from smbus2 import SMBus
from time import sleep

ARD_ADDR = 8 #set arduino address

i2c = SMBus(1)#initialize i2c bus to bus 1

def send_string(data, offset):
    '''
    Function to send a string to the arduino
    '''
    #convert the string to a list of ascii values list comprehension
    command = [ord(characters) for characters in data]
    #handle exception if i2c write fails
    try:
        #parameters are address of arduino, register to write to, and data to write
        i2c.write_i2c_block_data(ARD_ADDR, offset, command)
    except IOError:
        print("Could not write data to the Arduino.")
    #wait for a bit
    sleep(.1)
    

#run loop to send data to arduino
while(True):
    offset = int(input("Enter an offset (7 to quit): "))
    if(offset == 7):
        break
    string = input("enter a string of 32 characters or less: ")
    send_string(string, offset)
#data will appear in serial monitor on arduino will need to code on arduino side to interpret