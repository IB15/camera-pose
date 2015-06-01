# camera-pose
Extraction of 3D geometry from unsteady uncalibrated cameras.
This application is able to understand the pose of the cameras, given a set of user-generated images.
The inputs required to calibrate the cameras are two images in which the movement of the camera is small with respect to the distance between the camera itself and the scene. Once cameras have been calibrated, the inputs required are just
two photos taken from different position. The experimental results show an accurate
estimation of the relative camera position. If you provide the distance between 4 real scene
points, the algorithm is able to retrieve and show the position of the camera with respect to
the scene in 3D. If you don’t provide any additional information, the output will be a 2D map
of the position of one camera with respect to the other one.

This code is written and tested for MATLAB R2015a. For run the code you also need the toolbox:
- Image Processing Toolbox
- Computer Vision System Toolbox


**Guidline for using a set of user-generated uncalibrated sequence images**  
- This program is suitable for making a 3D camera pose, if you provide the distance between 4 real scene
points. If you don’t provide any additional information, the output will be a 2D map.
- This program available for only 2 photos, taken in 2 different position with the same camera. 
If there are more than 2 photos, you have to modify the main program.
- The sequence images must be close enough to create point correspondances between at least 6 points.



**INITIAL SET-UP**  
- The main is "Camera_Pose.m"
- Set all folders and files as Matlab directory path

**CAMERA CALIBRATION**  
- Select two image in such a way these has just a little rotation or translation of the camera with respect to the distance between the camera itself and the captured scene

**3D RECONSTRUCTION**  
- Select two image. If the real dimension of an object in the scene are known, you can reconstruct the 3D relative position of the cameras. For good results, it’s better to select an object aligned along the plane x-y. For example a book is a good calibration object; it is the smaller object you can use to obtain an accurate calibration. The bigger the object (bigger with respect to the area of the photo), the more accurate the estimation.

<br />
<br />
_**EXAMPLE**_  
- For run a quick test, start the main program "Camera_Pose.m"
- Select the images "lv1.jpg" and "lv2.jpg" for estimate the intrinsic parameter of the camera  (The camera used in this example is a SAMSUNG S4 mini and it's respective focal lenght is about 2700 pixels)
- Now for run a 3D Reconstruction select the images "book1.jpg" and "book2.jpg"
- When you are asked to add information click "Yes", and select the corners of the white book on the table, as in this figure:
![alt tag](http://s17.postimg.org/ytg288llb/select_point.jpg)
- The book dimensions are 260 x 195 mm
- At the end you will have a 3D map like this, the points in red are the point selected, and the green point is the origin

![alt tag](http://s17.postimg.org/yfeq8n1hr/results.jpg)
