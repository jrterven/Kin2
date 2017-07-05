% VIDEODEMOCOLLECTOR Illustrates how to use the Kin2Collector class which
%   is an interface for Kinect2 SDK functionality
%
% This is a rewrite of Juan's videoDemo.m to demonstrate usage of
% Kin2Collector features.
%
% Mark Tomaszewski, mark@mark-toma.com
%
% Adapted from original authors:
% Juan R. Terven, jrterven@hotmail.com
% Diana M. Cordova, diana_mce@hotmail.com
% 
% Citation:
% J. R. Terven, D. M. Cordova, "A Kinect 2 Toolbox for MATLAB", 
% https://github.com/jrterven/Kin2, 2016.
% 

function videoDemoCollector()

addpath('Mex'); clear all; close all;

% Create Kin2Collector object and initialize it
k2 = Kin2Collector('color','depth','infrared');
k2.pollData(); % populates relevant <data> properties

% initialize figure with three subplots
figure('CloseRequestFcn',@(src,evt)closeRequestFcn(src,evt,k2))
subplot(2,2,[1,2]); handles.hcolor    = imshow(k2.color,[]);       title('Color Source');
subplot(2,2,3);     handles.hdepth    = imshow(k2.depth,[0 4000]); title('Depth Source');    colormap('cool'); colorbar;
subplot(2,2,4);     handles.hinfrared = imshow(k2.infrared);       title('Infrared Source');

% Register callback with Kin2Collecter
k2.newDataCallback = @(src,evt)updateFigure(src,evt,handles);
k2.startStreaming(); % let her rip!

end

function updateFigure(src,~,handles)
% updateFigure  Called every time there's new data in Kin2Collector
set(handles.hdepth   ,'CData',src.depth);
set(handles.hcolor   ,'CData',src.color);
set(handles.hinfrared,'CData',imadjust(src.infrared,[],[],0.5));
drawnow; % need to flush graphics queue or it just locks up
end

function closeRequestFcn(src,~,k2)
% called when figure window is closed
k2.stopStreaming();
k2.delete();
delete(src);
end