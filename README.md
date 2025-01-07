# TrackMeasCalibration
This is the algorithm and test program for the transformation matrix calibration problem in the tracking-based vision measurement system.
In the combined tracking-based vision measurement system, the data transformation process can be expressed as Pg = Rtgi + (Rlt + Pli + tlt) + ttgi, where Pli is the local data, Pg is the combined global data, Rtgi, and ttgi are the rotation and translation vector of the local target pose, Rlt and tlt are the transformation matrix between the local vision system and the target. Rlt and tlt are fixed and need to be calibrated.
This project includes the following files:
1. trackcalibfuns.h-Header file. The declaration of the functions.
2. trackcalibfuns.cpp-function definitions.
3. main.cpp- a test program
4. .txt files- input test files.
5. imgs-The images collected by global and local vision system are stored.

Dependencies: OpenCV and Eigen
