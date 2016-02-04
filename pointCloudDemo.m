% POINTCLOUDDEMO Illustrates how to use the Kin2 class to get the
% pointcloud in camera space
%
% Juan R. Terven, jrterven@hotmail.com
% Diana M. Cordova, diana_mce@hotmail.com
% 
% Citation:
% J. R. Terven, D. M. Cordova, "A Kinect 2 Toolbox for MATLAB", 
% https://github.com/jrterven/Kin2, 2016.

addpath('Mex');
clear all
close all

% Create Kinect 2 object and initialize it
% Select sources as input parameters.
% Available sources: 'color', 'depth', 'infrared', 'body_index', 'body',
% 'face' and 'HDface'
k2 = Kin2('depth');

% images sizes
depth_width = 512; depth_height = 424; outOfRange = 4000;


% Create matrices for the images
depth = zeros(depth_height,depth_width,'uint16');
pointCloud = zeros(depth_height*depth_width,3);

% depth stream figure
figure, h1 = imshow(depth,[0 outOfRange]);
title('Depth Source (press q to exit)')
colormap('Jet')
colorbar
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress

% point cloud figure
figure, hpc = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'.');
title('Point Cloud (press q to exit)')
axis([-3 3 -3 3 0 4])
xlabel('X'), ylabel('Y'), zlabel('Z');
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
        % Copy data to Matlab matrices
        depth = k2.getDepth;

        % update depth figure
        depth(depth>outOfRange) = outOfRange; % truncate depht
        set(h1,'CData',depth); 
        
        pointCloud = k2.getPointCloud;
        set(hpc,'XData',pointCloud(:,1),'YData',pointCloud(:,2),'ZData',pointCloud(:,3));

    end
    
    % If user presses 'q', exit loop
    if ~isempty(k)
        if strcmp(k,'q'); 
            break;
        elseif strcmp(k,'p'); 
            pause;
        end;
    end
  
    pause(0.02)
end

% Close kinect object
k2.delete;

close all;
