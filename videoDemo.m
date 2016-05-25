% VIDEODEMO Illustrates how to use the Kin2 class which is an interface for
%   Kinect2 SDK functionality
%
% Juan R. Terven, jrterven@hotmail.com
% Diana M. Cordova, diana_mce@hotmail.com
% 
% Citation:
% Terven J. Cordova D.M., "Kin2. A Kinect 2 Toolbox for MATLAB", Science of
% Computer Programming.
% https://github.com/jrterven/Kin2, 2016.
% 

addpath('Mex');
clear all
close all

% Create Kinect 2 object and initialize it
% Available sources: 'color', 'depth', 'infrared', 'body_index', 'body',
% 'face' and 'HDface'
k2 = Kin2('color','depth','infrared');

% images sizes
depth_width = 512; depth_height = 424; outOfRange = 4000;
color_width = 1920; color_height = 1080;

% Color image is to big, let's scale it down
colorScale = 1;

% Create matrices for the images
depth = zeros(depth_height,depth_width,'uint16');
infrared = zeros(depth_height,depth_width,'uint16');
color = zeros(color_height*colorScale,color_width*colorScale,3,'uint8');
depthColor = zeros(depth_height,depth_width,3,'uint8');

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

% infrared stream figure
figure, h3 = imshow(infrared);
title('Infrared Source (press q to exit)');
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress

% depthColor stream figure
figure, h4 = imshow(depthColor,[]);
title('Color2DephAligned (press q to exit)');
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
        color = k2.getColor;
        infrared = k2.getInfrared;
        depthColor = k2.getAlignColor2Depth;               
        
        % update depth figure
        depth(depth>outOfRange) = outOfRange; % truncate depht
        set(h1,'CData',depth); 

        % update color figure
        color = imresize(color,colorScale);
        set(h2,'CData',color); 

        % update infrared figure
        %infrared = imadjust(infrared,[0 0.2],[0.5 1]);
        infrared = imadjust(infrared,[],[],0.5);
        set(h3,'CData',infrared); 
        
        set(h4,'CData',depthColor);
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
