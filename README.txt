C++ wrapper functions for the Microsoft Kinect 2, using Microsoft SDK.

Requirements:
- Kinect2 SDK. http://www.microsoft.com/en-us/download/details.aspx?id=44561
- Visual Studio 2012 or newer compiler
- MATLAB 2013a or newer (for Visual Studio 2012 support)
- MATLAB 2015b or newer for pointCloudDemo2, which uses MATLAB's built-in pointCloud object

Usage:
1) Set the compiler using mex -setup C++
2) Open compile_cpp_files and set the include and lib paths of Kinect2 SDK (see the provided paths)
3) Add to the windows path the bin directory containing the Kinect20.Fusion.dll and Kinect20.Face.dll 
   For example: C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\bin
4) If you modify Windows path, close Matlab and open it again in order to detect the changes.
3) Run compile_cpp_files.m

Demos:
1) videoDemo.m: displays depth, color, and infrared video.
2) mappingDemo.m: displays depth and color video, and allows to map points from one image to the other (See usage comments at the beginning of the script).
3) mapping2CamDemo.m: displays depth and color and allows to map points from depth and color to camera space and viceversa.
4) bodyDemo.m: displays depth and color and the skeleton on both images
5) pointCloudDemo.m: displays depth and a colored point cloud on a scatter3 
6) pointCloudDemo2.m displays depth and a colored point cloud using MATLAB's built-in pointCloud object and pcshow. 
7) bodyIndexDemo.m: displays body index frames
8) faceDemo.m: detect and track faces showing the facial landmarks and face properties
9) faceHDDemo.m: detect and track faces showing the 17 animation units and the high definition model
10) faceHDDemo2.m: builds a face model for the user and track the faces using this model.
11) kinectFusionDemo.m: demonstrates the use of Kinect Fusion. This is still in BETA. Need fixing memory leakage in C++ causing MATLAB to crash on a second run.
12) calibrationDemo.m: obtain depth camera intrinsic parameters and color camera parameters.

Citation:
If you find this toolbox useful please cite:
Terven Juan, Cordova-Esparza Diana,  Kin2. A Kinect 2 Toolbox for MATLAB, 
Science of Computer Programming, 2016, http://dx.doi.org/10.1016/j.scico.2016.05.009