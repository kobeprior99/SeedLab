from smbus2 import SMBus
from time import sleep
import RPi.GPIO as GPIO
import LCD  # Assuming you're using an LCD library, for example, `lcd` or `Adafruit_CharLCD`

ARD_ADDR = 8

# Set up I2C and LCD
i2c = SMBus(1)
lcd = LCD.Adafruit_CharLCD()  # Initialize LCD, adjust if you're using a different library

# Set up the LCD screen
lcd.begin(16, 2)  # 16x2 LCD, adjust as per your LCD

while(True):
    # Input an integer to send
    integer_value = int(input("Enter an integer (0 to quit): "))
    if integer_value == 0:
        break
    
    try:
        # Send the integer to Arduino (convert integer to byte format)
        i2c.write_i2c_block_data(ARD_ADDR, 0, [integer_value])  # Send the integer with offset 0
        
        # Wait for a short period to give the Arduino time to process
        sleep(0.1)
        
        # Read the modified integer from the Arduino
        modified_int = i2c.read_byte_data(ARD_ADDR, 0)  # Read the modified integer

        print(f"Received from Arduino: {modified_int}")

        # Display the modified integer on the LCD
        lcd.clear()  # Clear any previous text
        lcd.set_cursor(0, 0)  # Set cursor to the top-left
        lcd.message("Modified Int:")
        lcd.set_cursor(0, 1)  # Move to the next line
        lcd.message(str(modified_int))

    except IOError:
        print("Could not write data to the Arduino.")
    sleep(0.1)
