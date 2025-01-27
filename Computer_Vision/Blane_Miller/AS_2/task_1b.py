from smbus2 import SMBus
from time import sleep
import board
import time
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# I2C Setup
ARD_ADDR = 8
i2c = SMBus(1)

# LCD Setup (adjust pins based on your wiring)
lcd_columns = 16  # Number of columns in your LCD
lcd_rows = 2      # Number of rows in your LCD


i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

while True:
    # Input an integer to send
    try:
        integer_value = int(input("Enter an integer (0 to quit): "))
        if integer_value == 0:
            break

        # Send the integer to Arduino
        i2c.write_byte(ARD_ADDR,0,integer_value)

        # Wait for the Arduino to process the integer
        sleep(0.1)

        # Read the modified integer from the Arduino
        modified_int = i2c.read_byte(ARD_ADDR)

        print(f"Received from Arduino: {modified_int}")

        # Display the modified integer on the LCD
        lcd.clear()  # Clear any previous text
        lcd.message = f"Modified Int:\n{modified_int}"

    except IOError:
        print("I/O error occurred. Could not communicate with Arduino.")
    except ValueError:
        print("Invalid input. Please enter an integer.")
    sleep(0.1)
