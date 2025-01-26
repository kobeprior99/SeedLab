from smbus2 import SMBus
from time import sleep
import board
import digitalio
import adafruit_character_lcd.character_lcd as characterlcd

# I2C Setup
ARD_ADDR = 8
i2c = SMBus(1)

# LCD Setup (adjust pins based on your setup)
lcd_columns = 16  # Number of columns in your LCD
lcd_rows = 2      # Number of rows in your LCD

# Set GPIO pins (update these based on your wiring)
lcd_rs = digitalio.DigitalInOut(board.D4)
lcd_en = digitalio.DigitalInOut(board.D17)
lcd_d4 = digitalio.DigitalInOut(board.D18)
lcd_d5 = digitalio.DigitalInOut(board.D27)
lcd_d6 = digitalio.DigitalInOut(board.D22)
lcd_d7 = digitalio.DigitalInOut(board.D23)

# Initialize the LCD
lcd = characterlcd.Character_LCD_Mono(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows)

while True:
    # Input an integer to send
    integer_value = int(input("Enter an integer (0 to quit): "))
    if integer_value == 0:
        break

    try:
        # Send the integer to Arduino
        i2c.write_i2c_block_data(ARD_ADDR, 0, [integer_value])  # Send the integer with offset 0
        
        # Wait for the Arduino to process
        sleep(0.1)
        
        # Read the modified integer from Arduino
        modified_int = i2c.read_byte_data(ARD_ADDR, 0)  # Read the modified integer

        print(f"Received from Arduino: {modified_int}")

        # Display the modified integer on the LCD
        lcd.clear()  # Clear the LCD screen
        lcd.message = f"Modified Int:\n{modified_int}"

    except IOError:
        print("Could not write data to the Arduino.")
    sleep(0.1)
