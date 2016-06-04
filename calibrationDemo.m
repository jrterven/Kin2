% CALIBRATIONDEMO Illustrates how to use the Kin2 class to obtain the
% cameras' intrinsic parameters.
%
% Usage: 
%   press 'd' to obtain the depth camera intrinsics.
%   press 'c' to obtain the color camera intrinsics.
%
% Juan R. Terven, jrterven@hotmail.com
% Diana M. Cordova, diana_mce@hotmail.com
% 
% Citation:
% Terven Juan. Cordova-Esparza Diana, "Kin2. A Kinect 2 Toolbox for MATLAB", Science of
% Computer Programming, 2016. DOI: http://dx.doi.org/10.1016/j.scico.2016.05.009
%
% https://github.com/jrterven/Kin2, 2016.
% 

addpath('Mex');
clear all
close all

% Create Kinect 2 object and initialize it
% Available sources: 'color', 'depth', 'infrared', 'body_index', 'body',
% 'face' and 'HDface'
k2 = Kin2('color','depth');

% images sizes
depth_width = 512; depth_height = 424; outOfRange = 4000;
color_width = 1920; color_height = 1080;

% Color image is to big, let's scale it down
colorScale = 0.4;

% Create matrices for the images
depth = zeros(depth_height,depth_width,'uint16');
color = zeros(color_height*colorScale,color_width*colorScale,3,'uint8');

% depth stream figure
figure, h1 = imshow(depth,[0 outOfRange]);
title('Depth Source (press q to exit)')
colormap('Jet')
colorbar
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress

% color stream figure
figure, h2 = imshow(color,[]);
title('Color Source (press q to exit)');
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress

% Loop until pressing 'q' on any figure
k=[];

disp(' '); disp('Usage:');
disp('Press d to obtain depth camera intrinsic parameters')
disp('Press c to calibrate color camera')
disp('Press q on any figure to exit')

while true
    % Get frames from Kinect and save them on underlying buffer
    validData = k2.updateData;
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices
        depth = k2.getDepth;
        color = k2.getColor;

        % update depth figure
        depth(depth>outOfRange) = outOfRange; % truncate depht
        set(h1,'CData',depth); 

        % update color figure
        color = imresize(color,colorScale);
        set(h2,'CData',color); 

    end
    
    % If user presses 'q', exit loop
    if ~isempty(k)
        if strcmp(k,'q') 
            break; 
        elseif strcmp(k,'d')
            calib = k2.getDepthIntrinsics;
            disp(' ');
            disp('------------ Depth Intrinsics ------------')
            disp(['Focal Length X: ' num2str(calib.FocalLengthX)]);
            disp(['Focal Length Y: ' num2str(calib.FocalLengthY)]);
            disp(['Principal Point X: ' num2str(calib.PrincipalPointX)]);
            disp(['Principal Point Y: ' num2str(calib.PrincipalPointY)]);
            disp(['Radial Distortion 2nd order: ' num2str(calib.RadialDistortionSecondOrder)]);
            disp(['Radial Distortion 4th order: ' num2str(calib.RadialDistortionFourthOrder)]);
            disp(['Radial Distortion 6th order: ' num2str(calib.RadialDistortionSixthOrder)]);
            disp('--------------------------------------------');
            k = [];
        elseif strcmp(k,'c')
            calib = k2.getColorCalib;
            disp(' ');
            disp('------------ Color Camera Parameters ------------')
            disp(['Focal Length X: ' num2str(calib.FocalLengthX)]);
            disp(['Focal Length Y: ' num2str(calib.FocalLengthY)]);
            disp(['Principal Point X: ' num2str(calib.PrincipalPointX)]);
            disp(['Principal Point Y: ' num2str(calib.PrincipalPointY)]);
            disp('Rotation Wrt Depth camera:');
            disp(num2str(calib.Rotation));
            disp(['Translation x,y,z wrt depth camera(meters): ' num2str(calib.Translation)]);
            disp(['Radial Distortion 2nd order: ' num2str(calib.RadialDistortionSecondOrder)]);
            disp(['Radial Distortion 4th order: ' num2str(calib.RadialDistortionFourthOrder)]);
            disp(['Radial Distortion 6th order: ' num2str(calib.RadialDistortionSixthOrder)]);
            disp('--------------------------------------------');
            k = [];
        end;
    
    end
  
    pause(0.02)
end

% Close kinect object
k2.delete;

close all;
