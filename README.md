# Camera Calibration

You need to edit two files:
1. `aruco_detection_test.py`
2. `get_perspective_warping_with_aruco.py`

_
First, edit `get_perspective_warping_with_aruco.py`. When ran, this file will create a new window where you can pick the four points to use for camera calibration. All you need to edit in this code is adding an ordered list of the point you are selecting for calibration.

Next, in `aruco_detection_test.py` edit the function `image_frame_to_table_frame()` filled out. This function is what allows the robot to convert positions in an image to positions in the table frame.


We will update further instructions soon.

# Troubleshooting
If you get a missing package error try running 
``` pip install keyboard ```
``` pip install pydantic=1.10.9```

Detailed instructions at the [Lab-Code repository](https://github.com/ENME480/Lab-Code/blob/main/Final%20Project/Final_Project_Details.md)
