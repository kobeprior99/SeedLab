'''
*******************************************************************
* File Name         : distance.py

* Description       : Calculate the distance to an aruco marker using the size of the aruco marker in the image
*
* Supplementary File(s): cam_cal.py used to generate intrinsic camera parameters stored in calibration.pkl file   

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
- ArUco markers 2x2 inch per instructions on assignment

Example Execution: 
- Ensure calibration.pkl is available in the ComputerVision/Demo1 directory.
- Run the script using: python distance.py after navigating to the correct directory
- Place an ArUco marker in front of the camera
PLEASE NOTE: this is draft code and some documentation is sparse because it will not be submitted for a deliverable, the goal of this file was to quickly develop.
'''