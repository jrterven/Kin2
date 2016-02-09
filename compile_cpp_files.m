function compile_cpp_files
% compile_cpp_files will compile Kin2_mex.cpp
% which interfaces the Kinect2 SDK functions for Matlab.
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
%   4) Run this function
%
% Author: Juan R. Terven, jrterven@hotmail.com
IncludePath = 'C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\inc';
LibPath = 'C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\Lib\x64';

cd Mex
mex ('-compatibleArrayDims', '-v', 'Kin2_mex.cpp', ['-L' LibPath], ...
    '-lKinect20', '-lKinect20.Fusion', '-lKinect20.Face',['-I' IncludePath]);
