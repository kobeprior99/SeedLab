'''
*******************************************************************
* File Name         : arrow.py

* Description       : detect red-right arrow or green-left arrow and return 1 or 0 respectively
*
* Supplementary File(s): Computer_Vision/Demo1/cam_cal.py used to generate intrinsic camera parameters stored in calibration.pkl file  
* Kobe_Prior/AS2/assignment/task2.py to find contours

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
-print (ensure 2x2 inches): leftarrow.png, rightarrow.png

Example Execution: 
- Ensure calibration.pkl is available in the ComputerVision/Demo1 directory.
- Place an beacon marker in front of the camera
- Run the script using: python arrow.py after navigating to the correct directory
PLEASE NOTE: this is draft code and some documentation is sparse because it will not be submitted for a deliverable, the goal of this file was to quickly develop.
'''

