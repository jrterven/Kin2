% FACEHDDEMO Illustrates how to use the Kin2 object to get and display the
% HD face data
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
k2 = Kin2('color','HDface');

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

model = zeros(3,1347);
figure, hmodel = plot3(model(1,:),model(2,:),model(3,:),'.');
%axis([-1 1 -1 1 -1 1])
title('HD Face Model (press q to exit)')
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
        % Get color frame
        color = k2.getColor;
        
        % update color figure
        color = imresize(color,COL_SCALE);
        c.im = imshow(color, 'Parent', c.ax);
        
        % Get the HDfaces data
        % the output faces is a structure array with at most 6 faces. Each face has
        % the following fields:
        % - FaceBox: rectangle coordinates representing the face position in
        %   color space. [left, top, right, bottom].        
        % - FaceRotation: 1 x 3 vector containing: pitch, yaw, roll angles
        % - HeadPivot: 1 x 3 vector, computed center of the head, 
        %   which the face may be rotated around. 
        %   This point is defined in the Kinect body coordinate system. 
        % - AnimationUnits: 17 animation units (AUs). Most of the AUs are 
        %   expressed as a numeric weight varying between 0 and 1.
        %   For details see https://msdn.microsoft.com/en-us/library/microsoft.kinect.face.faceshapeanimations.aspx
        % - ShapeUnits: 94 hape units (SUs). Each SU is expressed as a 
        %   numeric weight that typically varies between -2 and +2.
        %   For details see https://msdn.microsoft.com/en-us/library/microsoft.kinect.face.faceshapedeformations.aspx
        % - FaceModel: 3 x 1347 points of a 3D face model computed by face capture
        faces = k2.getHDFaces('WithVertices','true');                    

        % Display the HD faces data and face model(1347 points):
        % Parameters: 
        % 1) image axes
        % 2) faces structure obtained with getFaces
        % 3) display HD face model vertices(1347 points)?
        % 4) display text information (animation units)?
        % 5) text font size in pixels
        k2.drawHDFaces(c.ax,faces,true,true,20);
             
        % Plot face model points
        if size(faces,2) > 0
             model = faces(1).FaceModel;
             set(hmodel,'XData',model(1,:),'YData',model(2,:),'ZData',model(3,:));
             view(0,90)
         end
    end
    
    % If user presses 'q', exit loop
    if ~isempty(k)
        if strcmp(k,'q'); break; end;
    end
  
    pause(0.02)
end

% Close kinect object
k2.delete;

%close all;
