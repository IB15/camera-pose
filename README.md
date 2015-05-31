# camera-pose
Extraction of 3D geometry from unsteady uncalibrated cameras.
This application is able to understand the pose of the cameras, given a set of user-generated images.
The inputs required to calibrate the cameras are two images in which the movement of the camera is small with respect to the distance between the camera itself and the scene. Once cameras have been calibrated, the inputs required are just
two photos taken from different position. The experimental results show an accurate
estimation of the relative camera position. If you provide the distance between 4 real scene
points, the algorithm is able to retrieve and show the position of the camera with respect to
the scene in 3D. If you don’t provide any additional information, the output will be a 2D map
of the position of one camera with respect to the other one.

This code is written and tested for MATLAB R2015a. For run the code you need also the toolbox:
- Image Processing Toolbox
- Computer Vision System Toolbox


Guidline for using a set of user-generated uncalibrated sequence images

1) This program is proper only for making a 3D camera pose, if you provide the distance between 4 real scene
points. If you don’t provide any additional information, the output will be a 2D map.
2) This program available for only 2 photos, taken in 2 different position with the same camera. 
If there are more than 2 photos, you have to modify the main program.
3) The sequence images must be close enough to create point correspondances between atleast 6 points.



INITIAL SET-UP

1.The main program is Camera_Pose.m
2.Set all folders and files as Matlab directory path

CAMERA CALIBRATION

Select two image. For good results these has to present just a little rotation or translation of the camera with respect to the distance between the camera itself and the captured scene

3D RECONSTRUCTION

If know the real dimension of a object in the scene you can reconstruct the 3D relative position of the cameras.
For good results, it’s better to select an object aligned alon gth plane x-y. For example a book is a good calibration object; it is the smaller object you can use to obtain an accurate calibration. The bigger the object (bigger with
respect to the area of the photo), the more accurate the estimation.

