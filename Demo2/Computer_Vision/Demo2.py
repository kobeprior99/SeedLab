'''
*******************************************************************
* File Name         : Demo2.py

* Description       : Interpret the angle of an Aruco marker, 
* the distance to the marker, and the color of the arrow next to the marker. 
* When this information is available send it to the LCD screen and arduino.
*
* Supplementary File(s): Computer_Vision/Demo1/cam_cal.py used to generate intrinsic camera parameters stored in calibration.pkl file 
* distance.py used to experiment with distance calculation function
* arrow.py used to experiment with arrow detection function  
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 
* 02/22/2025    Kobe Prior and Blane Miller Created File
******************************************************************
Hardware Setup: 
- Raspberry Pi
- Pi Camera
- I2C LCD Display
-I2C Arduino
-connect pin 3 on Pi to pin A4 on Arduino (SDA)
-connect pin 5 on Pi to pin A5 on Arduino (SCL)
-connect pin 6 on Pi to GND on Arduino
-print (ensure 2x2 inches): leftarrow.png, rightarrow.png

Example Execution: 
- Ensure calibration.pkl is available in working directory.
- Run the script using: python Demo2.py after navigating to the correct directory
- print 2x2 inch aruco markers and leftarrow.png and rightarrow.png
- Place an left or right 'beacon' 
'''
