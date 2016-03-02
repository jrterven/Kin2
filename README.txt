C++ wrapper functions for the Microsoft Kinect 2, using Microsoft SDK.

Requirements:
- Kinect2 SDK. http://www.microsoft.com/en-us/download/details.aspx?id=44561
- Visual Studio 2012 or newer compiler
- Matlab 2013a or newer (for Visual Studio 2012 support)

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
5) pointCloudDemo.m: displays depth and pointcloud
6) bodyIndexDemo.m: displays body index frames
7) faceDemo.m: detect and track faces showing the facial landmarks and face properties
8) faceHDDemo.m: detect and track faces showing the 17 animation units and the high definition model
9) kinectFusionDemo.m: demonstrates the use of Kinect Fusion. This is still in BETA. Need fixing memory leakage in C++ causing MATLAB to crash on a second run.
10) calibrationDemo.m: obtain cameras intrinsic parameters.