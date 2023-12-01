<div align="center">
<h1 align="center">
Camera Calibration
</h1>
</div>
<div align="center">
<h3>
<a href="https://research.tue.nl/en/persons/maarten-jongeneel">Maarten Jongeneel</a>
<br>
<br>
Extrinsic and Intrinsic camera calibration via OpenCV for poses obtained via OptiTrack Motion Capture system.
<br>
<br>
</h3>
</div>

# Requirements
 - MATLAB 2020a or later. 
 - Python 3

# Introduction
This project contains the code for intrinsic and extrinsic camera calibration. The intrinsic calibration is done via OpenCV in Python and obtains also the checkerboard to camera poses. Then, using the OptiTrack data, a calibration is performed in Matlab to obtain the camera to casing pose.


Table of content
================
- [Overview](#overview)
- [Installation](#installation)
- [Usage of the scripts](#usage-of-the-scripts)
- [Contact](#contact)


# Overview
# Installation
# Usage of the scripts

1. Make sure you have the images of a checkerboard (with .png extension) placed in the ```data``` folder. 
2. Make sure you have the OptiTrack .csv Take files in the ```data``` folder, named in order equal to the order of your images
3. In ```calibratecam.py```, change
    ```python
    sizex = 9
    sizey = 14
    sizes = 51.4 #size of one square in mm
    ```
    to the correct size of your checkerboard and the correct pattern (in this case 9x14 squares)
4. Run calibratecam.py by opening a terminal and running
    ```bash
    python calibratecam.py
    ```
    This will create the ```camera_poses.csv``` file containing the checkerboard-to-camera poses. It will also give you the intrinsics camera matrix, but this matrix is currently not stored anywhere.
5. Open Matlab, add the ```functions``` folder to your path, and change the lines
    ```matlab
    %Name of OptiTrack object (casing frame)
    ObjName = "RealSense002";
    ```
    to the correct name of the object you defined in OptiTrack (the casing frame).Then, run ```CalibrateCam.m```. This now gives you the matrix ```GH_A```, the transformation matrix from camera casing to camera sensor.
# Contact