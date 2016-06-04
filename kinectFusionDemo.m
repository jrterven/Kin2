% KINECTFUSIONDEMO Illustrates how to use the Kin2 to perform 3D
% reconstruction
%
% Note: You must add to the windows path the bin directory containing the 
%       Kinect20.Fusion.dll 
%       For example: C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\bin
%
% WARNING: KINECT FUSION FUNCTIONALITY IS STILL IN BETA
%          WE NEED TO FIX MEMORY LEAKAGE IN C++ CAUSING MATLAB TO CRASH AFTER
%          A SECOND RUN.
%
% Juan R. Terven, jrterven@hotmail.com
% Diana M. Cordova, diana_mce@hotmail.com
% 
% Citation:
% Terven Juan. Cordova-Esparza Diana, "Kin2. A Kinect 2 Toolbox for MATLAB", Science of
% Computer Programming, 2016. DOI: http://dx.doi.org/10.1016/j.scico.2016.05.009
%
% https://github.com/jrterven/Kin2, 2016.

addpath('../Mex');
clear all
close all

% Create Kinect 2 object and initialize it
% Select sources as input parameters.
% Available sources: 'color', 'depth', 'infrared', 'body_index', 'body',
% 'face' and 'HDface'
k2 = Kin2('color','depth');

k2.KF_init;

% images sizes
depth_width = 512; depth_height = 424; outOfRange = 4000;
color_width = 1920; color_height = 1080;

% Color image is to big, let's scale it down
colorScale = 0.4;

% Create matrices for the images
depth = zeros(depth_height,depth_width,'uint16');
volume = zeros(depth_height,depth_width,3,'uint8');
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

% volume stream figure
figure, h3 = imshow(volume,[]);
title('Volume Source (press q to exit)');
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress

% Loop until pressing 'q' on any figure
k=[];
disp('Press q on any figure to exit')
while true
    tic
    % Get frames from Kinect and save them on underlying buffer
    validData = k2.updateData;
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices
        depth = k2.getDepth;
        color = k2.getColor;
        
        k2.KF_update;
        volume = k2.KF_getVolumeImage;

        % update depth figure
        depth(depth>outOfRange) = outOfRange; % truncate depht
        set(h1,'CData',depth); 

        % update color figure
        color = imresize(color,colorScale);
        set(h2,'CData',color); 

        % update infrared figure
        set(h3,'CData',volume); 
    end
    
    % If user presses 'q', exit loop
    if ~isempty(k)
        if strcmp(k,'q'); break; end;
        if strcmp(k,'m'); 
            mesh = k2.KF_getMesh; 
            k=[];
        end;
    end
  
    pause(0.02)
end

% Close kinect object
k2.delete;

close all;
