% MAPPINGTOCAMDEMO Illustrates how to map points between depth and color images
%
% Usage:
%   - Press 'd' to select a point on the depth image. The selected point 
%     will be mapped from depth to camera and the resulting coordinates are
%     printed on command window. Then the camera coordinates are mapped
%     back to depth space and printed to command window.
%   - Press 'c' to select a point on the color image. The selected point 
%     will be mapped from color to camera and the resulting coordinates are
%     printed on command window.  Then the camera coordinates are mapped
%     back to color space and printed to command window.
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
%
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
color_width = 1920; color_height = 1080;

% Color image is to big, let's scale it down
COL_SCALE = 0.5;

% Create matrices for the images
depth = zeros(depth_height,depth_width,'uint16');
color = zeros(color_height*COL_SCALE,color_width*COL_SCALE,3,'uint8');

% Images used to draw the markers
depthAdditions = zeros(depth_height,depth_width,3,'uint8');
colorAdditions = zeros(color_height*COL_SCALE,color_width*COL_SCALE,3,'uint8');

% depth stream figure
h1 = figure;
hdepth = imshow(depth,[0 255]);
title('Depth Source (press q to exit)')
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress

% color stream figure
h2 = figure;
hcolor = imshow(color,[]);
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
        set(hdepth,'CData',depth8uc3 + depthAdditions); 

        % update color figure
        color = imresize(color,COL_SCALE);
        set(hcolor,'CData',color + colorAdditions);       
    end
    
    % If user presses 'd' enter to points selection mode on the depth image
    % If user presses 'c' enter to points selection mode on the color image
    % If user presses 'q', exit loop
    if ~isempty(k)
        if strcmp(k,'d')
            figure(h1);
            title('Clic the image to sample a point');
            
            % Grab 1 points
            [x,y] = ginput(1);
            disp('Input depth coordinates');
            disp([x y])
            % Draw the selected points in the depth image
            depthAdditions = insertMarker(depthAdditions,[x y],'Color','red');
        
            % Map the point from depth coordinates to camera coordinates
            % Input: 1 x 2 matrix (1 points, x,y)
            % Output: 1 x 3 matrix (1 point, x,y,z)
            camCoords = k2.mapDepthPoints2Camera([x y]);           
            
            disp('Mapped camera coordinates');
            disp(camCoords);
            
            % Map the resulting camera point back to depth space
            depthCoords = k2.mapCameraPoints2Depth(camCoords);
            disp('Mapped depth coordinates');
            disp(depthCoords);
                       
           k = [];
        elseif strcmp(k,'c')
            figure(h2);
            title('Clic the image to sample 5 points');
            
            % Grab 1 point
            [x,y] = ginput(1);
            disp('Input color coordinates');
            disp([x/COL_SCALE y/COL_SCALE]);
            
            % Draw the selected point in the color image
            colorAdditions = insertMarker(colorAdditions,[x y],'Color','green','Size',5);
                        
            % Map the points from color coordinates to camera coordinates
            % Input: 1 x 2 matrix (1 points, x,y)
            % Output: 1 x 3 matrix (1 point, x,y,z)
            camCoords = k2.mapColorPoints2Camera([x/COL_SCALE y/COL_SCALE]);
            
            disp('Mapped camera coordinates')
            disp(camCoords);
 
            % Map the resulting camera point back to color space
            colorCoords = k2.mapCameraPoints2Color(camCoords);
            disp('Mapped color coordinates');
            disp(colorCoords);
            
            k = [];
        end
         
        if strcmp(k,'q'); break; end;
    end
  
    pause(0.02)
end

% Close kinect object
k2.delete;

close all
