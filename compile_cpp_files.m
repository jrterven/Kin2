function compile_cpp_files
% compile_cpp_files compiles the Kin2 toolbox.
% The C++ code is located in 6 files:
%   Kin2.h:  Kin2 class definition.
%   Kin2_base.cpp: Kin2 class implementation of the base functionality including body data.
%   Kin2_mapping.cpp: Kin2 class implementation of the mapping functionality
%   Kin2_face.cpp: Kin2 class implementation of the Face and HD face processing.
%   Kin2_fusion.cpp: Kin2 class implementation of the 3D reconstruction.
%   Kin2_mex.cpp: MexFunction implementation.
%
% Requirements:
% - Kinect2 SDK. http://www.microsoft.com/en-us/download/details.aspx?id=44561
% - Visual Studio 2012 or newer compiler
% - Matlab 2013a or newer (in order to support Visual Studio 2012)
%
% Usage:
%   1) Set the compiler using mex -setup C++ (note it doesn't work with
%   compilers older than VS2012.
%   2) Set the IncludePath and LibPath variables in this file to the correct locations 
%   (see example below)
%   3) Add to the windows path the bin directory containing the 
%      Kinect20.Fusion.dll and Kinect20.Face.dll 
%      For example: C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\bin
%   4) Close Matlab and open it again.
%   5) Run this function.
%
% Author: Juan R. Terven, jrterven@hotmail.com
IncludePath = 'C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\inc';
LibPath = 'C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\Lib\x64';

cd Mex
mex ('-compatibleArrayDims', '-v', 'Kin2_mex.cpp', 'Kin2_base.cpp', ...
    'Kin2_mapping.cpp','Kin2_face.cpp','Kin2_fusion.cpp', ...
    ['-L' LibPath],'-lKinect20', '-lKinect20.Fusion', '-lKinect20.Face',['-I' IncludePath]);
