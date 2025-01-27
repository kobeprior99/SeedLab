from smbus2 import SMBus
from time import sleep

ARD_ADDR = 8
i2c = SMBus(1)

while True:
    try:
        integer_value = int(input("Enter an integer (0 to quit): "))
        if integer_value == 0:
            break

        print(f"Sending: {integer_value}")
        i2c.write_byte(ARD_ADDR, integer_value)
        sleep(0.1)
        response = i2c.read_byte(ARD_ADDR)
        print(f"Received from Arduino: {response}")

    except IOError:
        print("I/O error occurred. Could not communicate with Arduino.")
    except ValueError:
        print("Invalid input. Please enter an integer.")
    sleep(0.1)
