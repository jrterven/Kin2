% BODYDEMO Illustrates how to use the Kin2 object to get and draw the
% Skeleton data
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
k2 = Kin2('color','depth','body');

% images sizes
d_width = 512; d_height = 424; outOfRange = 4000;
c_width = 1920; c_height = 1080;

% Color image is to big, let's scale it down
COL_SCALE = 1.0;

% Create matrices for the images
depth = zeros(d_height,d_width,'uint16');
color = zeros(c_height*COL_SCALE,c_width*COL_SCALE,3,'uint8');

% depth stream figure
d.h = figure;
d.ax = axes;
d.im = imshow(zeros(d_height,d_width,'uint8'));
%hold on;

title('Depth Source (press q to exit)')
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress

% color stream figure
c.h = figure;
c.ax = axes;
c.im = imshow(color,[]);
title('Color Source (press q to exit)');
set(gcf,'keypress','k=get(gcf,''currentchar'');'); % listen keypress
%hold on

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

        % update depth figure
        depth8u = uint8(depth*(255/outOfRange));
        depth8uc3 = repmat(depth8u,[1 1 3]);
        d.im = imshow(depth8uc3, 'Parent', d.ax);

        %set(d.im,'CData',depth8uc3); 

        % update color figure
        color = imresize(color,COL_SCALE);
        c.im = imshow(color, 'Parent', c.ax);

        %set(c.im,'CData',color); 

        % Get 3D bodies joints 
        % Input parameter can be 'Quat' or 'Euler' for the joints
        % orientations.
        % getBodies returns a structure array.
        % The structure array (bodies) contains 6 bodies at most
        % Each body has:
        % -Position: 3x25 matrix containing the x,y,z of the 25 joints in
        %   camera space coordinates
        % - Orientation: 
        %   If input parameter is 'Quat': 4x25 matrix containing the 
        %   orientation of each joint in [x; y; z, w]
        %   If input parameter is 'Euler': 3x25 matrix containing the 
        %   orientation of each joint in [Pitch; Yaw; Roll] 
        % -TrackingState: state of each joint. These can be:
        %   NotTracked=0, Inferred=1, or Tracked=2
        % -LeftHandState: state of the left hand
        % -RightHandState: state of the right hand
        [bodies, fcp, timeStamp] = k2.getBodies('Quat');        
        
        % Number of bodies detected
        numBodies = size(bodies,2);
        %disp(['Bodies Detected: ' num2str(numBodies)])
       
        % Example of how to extract information from getBodies output.
       if numBodies > 0
            % first body info:
            %disp(bodies(1).TrackingState)
            %disp(bodies(1).RightHandState)
            %disp(bodies(1).LeftHandState)
            
            %disp('Right Hand Orientation') % see Kin2.m constants
            %disp(bodies(1).Orientation(:,k2.JointType_HandRight));   
            
            disp('Floor Clip Plane')
            disp(fcp);
            
            disp('Body Timestamp')
            disp(timeStamp);
        
            % To get the joints on depth image space, you can use:
            %pos2D = k2.mapCameraPoints2Depth(bodies(1).Position');
        end
         
        %To get the joints on color image space, you can use:
        %pos2D = k2.mapCameraPoints2Color(bodies(1).Position');

        % Draw bodies on depth image
        % Parameters: 
        % 1) image axes
        % 2) bodies structure
        % 3) Destination image (depth or color)
        % 4) Joints' size (circle raddii)
        % 5) Bones' Thickness
        % 6) Hands' Size
        k2.drawBodies(d.ax,bodies,'depth',5,3,15);
        
        % Draw bodies on color image
        k2.drawBodies(c.ax,bodies,'color',10,6,30);
        
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
