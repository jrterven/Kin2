% POINTCLOUDDEMO Illustrates how to use the Kin2 class to get the
% pointcloud in camera space
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
k2 = Kin2('color', 'depth');

% images sizes
depth_width = 512; depth_height = 424; outOfRange = 4000;

% Create matrices for the images
depth = zeros(depth_height,depth_width,'uint16');
pc = zeros(depth_height*depth_width,3);

% depth stream figure
figure, h1 = imshow(depth,[0 outOfRange]);
title('Depth Source (press q to exit)')
colormap('Jet')
colorbar
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress

% point cloud figure
figure
pcax = axes;
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress

% Loop until pressing 'q' on any figure
k=[];

disp('Press q on any figure to exit')
downsample = 2; % subsample pointcloud
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
        
        % Obtain the point cloud with color
        [pc, pcColors] = k2.getPointCloud('output','raw','color','true');
        pcColors = double(pcColors)/255.0;
        scatter3(pcax,pc(:,1),pc(:,2),pc(:,3),6,pcColors,'Marker','.');
        axis(pcax,[-3 3 -3 3 0 4])
        xlabel(pcax,'X'), ylabel(pcax,'Y'), zlabel(pcax,'Z');
        view(pcax,180,-90)

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

%close all;
