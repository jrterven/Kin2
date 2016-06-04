% FACEHDDEMO2 Illustrates how to build a personal face model and use it to
% track a face.
% Quoting the High definition face tracking documentation: 
% https://msdn.microsoft.com/en-us/library/dn785525.aspx
% "The tracking quality increases if the face has been captured, 
% and the output of the capture used to initialize the face tracking. 
% This enables the face tracker to use the precise geometry of the face 
% instead of an average geometry, and results in more accurate 
% characterization of face motions."
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

% Loop until pressing 'q' on any figure
k=[];

faceModelReady = false;

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
        
        % Build a face model
        % Follow the instructions displayed on Command Window.
        % The user should be in front of the Kinect V2 in an adecuate
        % distance (> 1m) in order to track the body and face.
        % Move the face to the sides and up and down slowly to capture the
        % whole face and produce the model.
        % Input parameters: CollectionStatus and CaptureStatus display
        % status information on the Command Window
        if ~faceModelReady
            faceModelReady = k2.buildHDFaceModels('CollectionStatus','true','CaptureStatus','true');
        else        
            % Once the model is built, track the face using the model.
            % The model is internally attached to the face.
            faces = k2.getHDFaces('WithVertices','true');                    
            k2.drawHDFaces(c.ax,faces,true,true,20);
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
