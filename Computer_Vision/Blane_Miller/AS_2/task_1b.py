'''
*******************************************************************
* File Name         : task_1b.py
* Description       : have the pi send an integer to the arduino, 
send back that plus 100 to the pi, 
have the lcd screen display that modded integer
*                    
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 01/25/2025	Blane Miller	Created File
*
******************************************************************
Hardware Setup: Power on the Pi -> either connect peripherials or connect via PiConnect or other VNC service --> connect web cam
Example Excecution: -> open terminal -> navigate to the directory where the task_1b.py file is located using cd command ->
run the python file using 'python task1.py' command and point the camera towards the something that may contain green.
'''
from smbus2 import SMBus
from time import sleep
import board
import time
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# I2C Setup
ARD_ADDR = 8
i2c1 = SMBus(1)

# LCD Setup 
lcd_columns = 16  
lcd_rows = 2      
i2c2 = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c2, lcd_columns, lcd_rows)

while True:
    # Input an integer to send
    try:
        integer_value = int(input("Enter an integer (0 to quit): "))
        if integer_value == 0:
            break

        # Send the integer to Arduino
        i2c1.write_byte(ARD_ADDR,integer_value)

        # Wait for the Arduino to process the integer
        sleep(0.1)

        # Read the modified integer from the Arduino
        modified_int = i2c1.read_byte(ARD_ADDR)

        print(f"Received from Arduino: {modified_int}")

        # Display the modified integer on the LCD
        lcd.clear()  
        lcd.message = f"Modified Int:\n{modified_int}"

    except IOError:
        print("I/O error occurred. Could not communicate with Arduino.")
    except ValueError:
        print("Invalid input. Please enter an integer.")
    sleep(0.1)
