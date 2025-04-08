'''
*******************************************************************
* File Name         : refined_arrow.py

* Description       : Changed Demo2.py to use ArUco marker detection and adjust fov for arrow detection.
*
* Supplementary File(s): 
*
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 
* 4/7/2025 Blane Modified arrow detection to use the new ArUco marker detection and adjust fov.
******************************************************************
Hardware Setup: 
- Raspberry Pi
- Pi Camera
- I2C Arduino
- Connections:
    - Connect pin 3 on Pi to pin A4 on Arduino (SDA).
    - Connect pin 5 on Pi to pin A5 on Arduino (SCL).
    - Connect pin 6 on Pi to GND on Arduino.
- Print (ensure 2x2 inches): leftarrow.png, rightarrow.png.

Example Execution: 
- Ensure calibration.pkl is available in the working directory.
- Run the script using: python Demo2.py after navigating to the correct directory.
- Print 2x2 inch ArUco markers and leftarrow.png/rightarrow.png.
- Place a left or right 'beacon' 5 to 6 feet away from the robot, aligned with its axis of rotation, and power on the robot.
'''
