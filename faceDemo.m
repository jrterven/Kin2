% FACEDEMO Illustrates how to use the Kin2 object to get and draw the
% face data
%
% Note: You must add to the windows path the bin directory containing the 
%       Kinect20.Face.dll. 
%       For example: C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\bin
%
% Juan R. Terven, jrterven@hotmail.com
% Diana M. Cordova, diana_mce@hotmail.com
% 
% Citation:
% Terven Juan. Cordova-Esparza Diana, "Kin2. A Kinect 2 Toolbox for MATLAB", Science of
% Computer Programming, 2016. DOI: http://dx.doi.org/10.1016/j.scico.2016.05.009
%
% https://github.com/jrterven/Kin2, 2016.

addpath('Mex');
clear all
close all

% Create Kinect 2 object and initialize it
% Available sources: 'color', 'depth', 'infrared', 'body_index', 'body',
% 'face' and 'HDface'
k2 = Kin2('color','face');

% images sizes
c_width = 1920; c_height = 1080;

% Color image is to big, let's scale it down
COL_SCALE = 1.0;

% Create matrices for the images
color = zeros(c_height*COL_SCALE,c_width*COL_SCALE,3,'uint8');

% color stream figure
c.h = figure;
c.ax = axes;
c.im = imshow(color,[]);
title('Color Source (press q to exit)');
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress

% Loop until pressing 'q' on any figure
k=[];

disp('Press q on any figure to exit')
while true
    % Get frames from Kinect and save them on underlying buffer
    validData = k2.updateData;
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Get color frame
        color = k2.getColor;
        
        % Get the faces data
        % faces is a structure array with at most 6 faces. Each face has
        % the following fields:
        % - FaceBox: rectangle coordinates representing the face position in
        %   color space. [left, top, right, bottom].
        % - FacePoints: 2 x 5 matrix representing 5 face landmarks: 
        %   left eye, right eye, nose, right and left mouth corners.
        % - FaceRotation: 1 x 3 vector containing: pitch, yaw, roll angles
        % - FaceProperties: 1 x 8 vector containing the detection result of
        %   each of the face properties. 
        %   The face properties are:
        %   Happy, Engaged, WearingGlasses, LeftEyeClosed, RightEyeClosed,
        %   MouthOpen, MouthMoved, LookingAway
        %   The detection results are: 
        %   Unknown = 0, No = 1, Maybe = 2, Yes = 3;
        faces = k2.getFaces;                

        % update color figure
        color = imresize(color,COL_SCALE);
        c.im = imshow(color, 'Parent', c.ax);

        % Display the faces data:
        % Parameters: 
        % 1) image axes
        % 2) faces structure obtained with getFaces
        % 3) face landmarks size (radius)
        % 4) display text information?
        % 5) information font size in pixels
        k2.drawFaces(c.ax,faces,5,true,20);
             
    end
    
    % If user presses 'q', exit loop
    if ~isempty(k)
        if strcmp(k,'q'); break; end;
    end
  
    pause(0.02)
end

% Close kinect object
k2.delete;

close all;
