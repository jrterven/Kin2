% DEMOALL Illustrates how to use most of the Kin2 capabilities
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

% Specify the data sources
k2=Kin2('color','depth','infrared','body_index','body', ...
'face','HDface');

depth = zeros(424,512,'uint16');
color = zeros(1080,1920,3,'uint8');
% Matlab figures: depth, color, infrared, body index, volume
d.h=figure; d.ax=axes; d.im=imshow(depth,[0 4000]);
c.h=figure; c.ax=axes; c.im=imshow(color,[]);
infra.h = figure; infra.ax = axes; infra.im = imshow(depth);
bi.h = figure; bi.ax = axes; bi.im = imshow(depth);
pc.h = figure; pc.ax = axes;
v.h = figure; v.ax = axes; v.im = imshow(depth);

% Initialize Kinect Fusion engine with a reconstruction 
% of 1.5 wide and a resolution of 4mm/voxel
k2.KF_init(256,384,384,384,true);

for i=1:200
    % Fetch data from Kinect and save them in k2 object
    validData = k2.updateData;

    if validData  % valid frame?
        % Get frames
        depth = k2.getDepth;
        color = k2.getColor;
        infrared = k2.getInfrared;
        bodyIndex = k2.getBodyIndex;
        [pcd, pcCol] = k2.getPointCloud('color','true');
        pcCol = double(pcCol)/255.0;
        
        % Display frames
        imshow(depth,'Parent', d.ax)
        imshow(color,'Parent', c.ax)
        imshow(infrared,'Parent', infra.ax)
        imshow(bodyIndex,'Parent', bi.ax)        
        scatter3(pc.ax,pcd(:,1),pcd(:,2),pcd(:,3),6,pcCol,'Marker','.');
        
        % Get bodies data and display them
        bodies = k2.getBodies('Quat');
        k2.drawBodies(d.ax,bodies,'depth',5,3,15);
        k2.drawBodies(c.ax,bodies,'color',10,6,30); 
        
        % Get faces data and draw them on the color frame
        faces = k2.getFaces;
        k2.drawFaces(c.ax,faces,5,true,20);
        
        % Get HD faces data and draw them
        facesHD = k2.getHDFaces;
        k2.drawHDFaces(c.ax,faces,true,true,20);
        
        % Update 3D reconstruction and display it
        k2.KF_update;
        volume = k2.KF_getVolumeImage;
        imshow(volume,'Parent', v.ax)
                
    end
    pause(0.01)
end

k2.delete; % close Kinect connection