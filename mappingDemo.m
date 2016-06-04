% MAPPINGDEMO Illustrates how to map points between depth and color images
%
% Usage:
%   - Press 'd' to select 5 points on the depth image. The selected points 
%     will be mapped from depth to color and will be displayed on both 
%     images in red.
%   - Press 'c' to select 5 point on the color image. The selected points 
%     will be mapped from color to depth and will be displayed on both
%     images in green.
%   - Press 'q' to exit.
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

% Create a Kin2 object and initialize it
% Select sources as input parameters.
% Available sources: 'color', 'depth', 'infrared', 'body_index', 'body',
% 'face' and 'HDface'
k2 = Kin2('color','depth');

% images sizes
d_width = 512; d_height = 424; outOfRange = 4000;
c_width = 1920; c_height = 1080;

% Color image is to big, let's scale it down
COL_SCALE = 0.5;

% Create matrices for the images
depth = zeros(d_height,d_width,'uint16');
color = zeros(c_height*COL_SCALE,c_width*COL_SCALE,3,'uint8');

% Images used to draw the markers
depthAdditions = zeros(d_height,d_width,3,'uint8');
colorAdditions = zeros(c_height*COL_SCALE,c_width*COL_SCALE,3,'uint8');

% depth stream figure
d.h = figure;
d.ax = axes('units','pixels');
d.im = imshow(depth,[0 255]);
title('Depth Source (press q to exit)')
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress

% color stream figure
c.h = figure;
c.im = imshow(color,[]);
title('Color Source (press q to exit)');
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress


% Loop until pressing 'q' on any figure
k=[];

disp('Instructions:')
disp('Press d to select a point on the depth image')
disp('Press c to select a point on the color image')
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
        depth8u = uint8(depth*(255/outOfRange));
        depth8uc3 = repmat(depth8u,[1 1 3]);
        set(d.im,'CData',depth8uc3 + depthAdditions); 

        % update color figure
        color = imresize(color,COL_SCALE);
        set(c.im,'CData',color + colorAdditions);       
    end
    
    % If user presses 'd' enter to points selection mode on the depth image
    % If user presses 'c' enter to points selection mode on the color image
    % If user presses 'q', exit loop
    if ~isempty(k)
        if strcmp(k,'d')
            figure(d.h);
            title('Clic the image to sample 5 points');
            
            % Grab 5 points
            [x,y] = ginput(5);
            disp('Input depth coordinates');
            disp([x y])
            % Draw the selected points in the depth image
            depthAdditions = insertMarker(depthAdditions,[x y],'Color','red');
        
            % Using the mapping, map the points from depth coordinates to color coordinates
            % Input and output: n x 2 matrix (n points)
            colorCoords = k2.mapDepthPoints2Color([x y]);
            colorCoords = colorCoords * COL_SCALE; % scale the color coordinates
            
            disp('Output color coordinates');
            disp(colorCoords);
            
            % Draw the output coordinates on the color image
            colorAdditions = insertMarker(colorAdditions, colorCoords,'Color','red','Size',10); 
            
           k = [];
        elseif strcmp(k,'c')
            figure(c.h);
            title('Clic the image to sample 5 points');
            
            % Grab 5 points
            [x,y] = ginput(5);
            disp('Input color coordinates');
            disp([x y]);
            
            % Draw the selected points in the color image
            colorAdditions = insertMarker(colorAdditions,[x y],'Color','green','Size',5);
                        
            % Using the mapping, map the points from color coordinates to depth coordinates
            % Input and output: n x 2 matrix (n points)
            depthCoords = k2.mapColorPoints2Depth([x/COL_SCALE y/COL_SCALE]);
            
            disp('Output depth coordinates')
            disp(depthCoords);
            
            % Drae the output coordinates on the depth image
            depthAdditions = insertMarker(depthAdditions,depthCoords,'Color','green'); 
 
            
            k = [];
        end
         
        if strcmp(k,'q'); break; end;
    end
  
    pause(0.02)
end

% Close kinect object
k2.delete;

close all
