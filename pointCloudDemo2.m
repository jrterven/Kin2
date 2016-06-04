% POINTCLOUDDEMO2 Illustrates how to use the Kin2 class to obtain and display the
% pointcloud with color.
%
% Note: This demo uses the pointCloud object available introduced in MATLAB 2015b
%       Older versions of MATLAB will not recognize this object.
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
% Select sources as input parameters.
% Available sources: 'color', 'depth', 'infrared', 'body_index', 'body',
% 'face' and 'HDface'
k2 = Kin2('color','depth');

% images sizes
depth_width = 512; depth_height = 424; outOfRange = 4000;

% Create matrices for the images
depth = zeros(depth_height,depth_width,'uint16');
pc = pointCloud(zeros(depth_height*depth_width,3));


% depth stream figure
figure, h1 = imshow(depth,[0 outOfRange]);
title('Depth Source (close figure to exit)')
colormap('Jet')
colorbar

% point cloud figure
pcFig.h = figure;
pcFig.ax = pcshow(pc);

disp('Close any figure to exit')
downsample = 2; % subsample pointcloud

% Main Loop
while true
    % Get frames from Kinect and save them on underlying buffer
    validData = k2.updateData;
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices
        depth = k2.getDepth;

        % update depth figure
        depth(depth>outOfRange) = outOfRange; % truncate depht
        
        % Display the depth image, 
        % if the user closes the window, the program ends
        try
            set(h1,'CData',depth); 
        catch
            break; % break the main loop 
        end
          
        % Get the pointcloud with color from the Kinect
        % Select the output 'pointCloud' to use the MATLAB built-in
        % pointCloud object. 
        % For MATLAB versions older than 2015b, use 'output','raw' and use
        % scatter3 to plot the point cloud. See pointCloudDemo1.m
        pc = k2.getPointCloud('output','pointCloud','color','true');
        
        % Display the point cloud,
        % if the user closes the window, the program ends
        try
            pcshow(pc,'Parent',pcFig.ax,'VerticalAxis','Y');
            title(pcFig.ax,'Point Cloud');
            xlabel(pcFig.ax,'X'); ylabel(pcFig.ax,'Y'); zlabel(pcFig.ax,'Z');
            axis(pcFig.ax,[-4 4 -4 4 -4 4]);
        catch
            break; % break the main loop
        end
    end
  
    pause(0.02);
end

% Close kinect object
k2.delete;
