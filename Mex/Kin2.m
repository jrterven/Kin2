classdef Kin2 < handle
    %KIN2 MATLAB class wrapper to an underlying C++ Kin2 class
    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
        
        % Bodies colors
        bodyColors = ['r','b','g','y','m','c'];
    end
        
    properties (SetAccess = public)
        cDepthWidth     = 512;
        cDepthHeight    = 424;
        cColorWidth     = 1920;
        cColorHeight    = 1080;
        
        % Joints constants
        JointType_SpineBase     = 1;
        JointType_SpineMid      = 2;
        JointType_Neck          = 3;
        JointType_Head          = 4;
        JointType_ShoulderLeft	= 5;
        JointType_ElbowLeft     = 6;
        JointType_WristLeft     = 7;
        JointType_HandLeft      = 8;
        JointType_ShoulderRight	= 9;
        JointType_ElbowRight	= 10;
        JointType_WristRight	= 11;
        JointType_HandRight     = 12;
        JointType_HipLeft       = 13;
        JointType_KneeLeft      = 14;
        JointType_AnkleLeft     = 15;
        JointType_FootLeft      = 16;
        JointType_HipRight      = 17;
        JointType_KneeRight     = 18;
        JointType_AnkleRight	= 19;
        JointType_FootRight     = 20;
        JointType_SpineShoulder	= 21;
        JointType_HandTipLeft	= 22;
        JointType_ThumbLeft     = 23;
        JointType_HandTipRight	= 24;
        JointType_ThumbRight	= 25;
        JointType_Count         = 25;
        
        % Hand State constants
        HandState_Unknown       = 0,
        HandState_NotTracked	= 1,
        HandState_Open          = 2,
        HandState_Closed        = 3,
        HandState_Lasso         = 4
        
        % Face detection constants
        DetectionResult_Unknown = 0;
        DetectionResult_No      = 1;
        DetectionResult_Maybe	= 2;
        DetectionResult_Yes     = 3;
        
        % Face properties
        faceProperties = cell(8,1);
        
        % Face Animation units
        faceAnimationUnits = cell(17,1);
    end
    
    methods        
        function this = Kin2(varargin)
            % Constructor - Create a new C++ class instance 
            
            % Fill the face properties descriptions
            this.faceProperties(1) = cellstr('Happy');
            this.faceProperties(2) = cellstr('Engaged');
            this.faceProperties(3) = cellstr('Wearing Glasses');
            this.faceProperties(4) = cellstr('Left eye closed');
            this.faceProperties(5) = cellstr('Right eye closed');
            this.faceProperties(6) = cellstr('Mouth open');
            this.faceProperties(7) = cellstr('Mouth moved');
            this.faceProperties(8) = cellstr('Looking away');
            
            % Fill the face animation units descriptions
            this.faceAnimationUnits(1) = cellstr('JawOpen');
            this.faceAnimationUnits(2) = cellstr('LipPucker');
            this.faceAnimationUnits(3) = cellstr('JawSlideRight');
            this.faceAnimationUnits(4) = cellstr('LipStretcherRight');
            this.faceAnimationUnits(5) = cellstr('LipStretcherLeft');
            this.faceAnimationUnits(6) = cellstr('LipCornerPullerLeft');
            this.faceAnimationUnits(7) = cellstr('LipCornerPullerRight');
            this.faceAnimationUnits(8) = cellstr('LipCornerDepressorLeft');
            this.faceAnimationUnits(9) = cellstr('LipCornerDepressorRight');
            this.faceAnimationUnits(10) = cellstr('LeftcheekPuff');
            this.faceAnimationUnits(11) = cellstr('RightcheekPuff');
            this.faceAnimationUnits(12) = cellstr('LefteyeClosed');
            this.faceAnimationUnits(13) = cellstr('RighteyeClosed');
            this.faceAnimationUnits(14) = cellstr('RighteyebrowLowerer');
            this.faceAnimationUnits(15) = cellstr('LefteyebrowLowerer');
            this.faceAnimationUnits(16) = cellstr('LowerlipDepressorLeft');
            this.faceAnimationUnits(17) = cellstr('LowerlipDepressorRight');
            
            
            % Get the flags
            depth = ismember('depth',varargin);
            color = ismember('color',varargin);
            infrared = ismember('infrared',varargin);
            bodyIndex = ismember('body_index',varargin);
            body = ismember('body',varargin);
            face = ismember('face',varargin);
            hd_face = ismember('HDface',varargin);
            flags = uint16(0);
            
            % Extract flags values (from Kinect.h)
%             FrameSourceTypes_None	= 0,
%             FrameSourceTypes_Color	= 0x1,
%             FrameSourceTypes_Infrared	= 0x2,
%             FrameSourceTypes_Depth	= 0x8,
%             FrameSourceTypes_BodyIndex	= 0x10,
%             FrameSourceTypes_Body	= 0x20,
            if color, flags = flags + 1; end
            if infrared, flags = flags + 2; end
            if depth, flags = flags + 8; end
            if bodyIndex, flags = flags + 16; end
            if body, flags = flags + 32; end    
            if face, flags = flags + 128; end
            if hd_face, flags = flags + 256; end
            this.objectHandle = Kin2_mex('new', flags);
        end
                
        function delete(this)
            % Destructor - Destroy the C++ class instance
            Kin2_mex('delete', this.objectHandle);
        end
        
        %% video Sources        
        function varargout = updateData(this, varargin)
            % updateData - Update video frames. Call this function before
            % grabbing the frames.
            % Return: a flag indicating valid data.
            [varargout{1:nargout}] = Kin2_mex('updateData', this.objectHandle, varargin{:});
        end
                
        function varargout = getDepth(this, varargin)
            % getDepth - return depth frame from Kinect2. You must call
            % updateKin2 before and verify that there is valid data.
            % See videoDemo.m
            [varargout{1:nargout}] = Kin2_mex('getDepth', this.objectHandle, varargin{:});
        end
                
        function varargout = getColor(this, varargin)
            % getColor - return color frame from Kinect2. You must call
            % updateKin2 before and verify that there is valid data.
            % See videoDemo.m
            [varargout{1:nargout}] = Kin2_mex('getColor', this.objectHandle, varargin{:});
        end
                
        function varargout = getInfrared(this, varargin)
            % getInfrared - return infrared frame from Kinect2. You must call
            % updateKin2 before and verify that there is valid data.
            % See videoDemo.m
            [varargout{1:nargout}] = Kin2_mex('getInfrared', this.objectHandle, varargin{:});
        end
        
        function varargout = getBodyIndex(this, varargin)
            % getBodyIndex - return body index frame from Kinect2. You must call
            % updateKin2 before and verify that there is valid data.
            % See bodyIndexDemo.m
            [varargout{1:nargout}] = Kin2_mex('getBodyIndex', this.objectHandle, varargin{:});
        end
        
        function varargout = getPointCloud(this, varargin)
            % getPointCloud - return nx3 camera space values from Kinect2. You must call
            % updateKin2 before and verify that there is valid data.
            % See pointCloudDemo.m
            [varargout{1:nargout}] = Kin2_mex('getPointCloud', this.objectHandle, varargin{:});
        end
        
        function varargout = getFaces(this, varargin)
            % getFaces - return structure array with the properties for
            % each found face. You must call update before and verify that
            % there is valid data.
            % The returned structure has the following fields for each
            % detected face:
            % - FaceBox: rectangle coordinates representing the face position in
            %   color space. [left, top, right, bottom].
            % - FacePoints: 2 x 5 matrix representing 5 face landmarks: 
            %   left eye, right eye, nose, right and left mouth corners.
            % - FaceRotation: 1 x 3 vector containing: pitch, yaw, roll angles
            % - FaceProperties: 1 x 8 vector containing the detection result of
            %   each of the face properties. 
            %   The face properties are:
            %   Happy, Engaged, WearingGlasses, LeftEyeClosed, RightEyeClosed,
            %   MouthOpen, MouthMoved, LookingAway
            %   The detection results are: 
            %   Unknown = 0, No = 1, Maybe = 2, Yes = 3;
            % See faceDemo.m
            [varargout{1:nargout}] = Kin2_mex('getFaces', this.objectHandle, varargin{:});
        end
        
        function varargout = getHDFaces(this, varargin)
            % getHDFaces - return structure array with the following information:
            % 
            % You must call update before and verify that there is valid data
            % The returned stucture has the following fields for each
            % detected face:
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
            % See faceHDDemo.m
            [varargout{1:nargout}] = Kin2_mex('getHDFaces', this.objectHandle, varargin{:});
        end
        
        %% Depth mappings        
        function varargout = mapDepthPoints2Color(this, varargin)
            % mapDepthPoints2Color - map the input points from depth 
            % coordinates to color coordinates
            % Input and output: n x 2 matrix (n points)
            % See mappingDemo.m
            inputSize = size(varargin{1},1);
            [varargout{1:nargout}] = Kin2_mex('mapDepthPoints2Color', this.objectHandle, varargin{1},uint32(inputSize));
        end
        
                %% Depth mappings        
        function varargout = mapDepthFrame2Color(this, varargin)
            % mapDepthFrame2Color - map an input depth image
            % to color coordinates.
            % Input: cDepthHeight x cDepthWidth matrix
            % Output: cDepthHeight x cDepthWidth mapping matrix
            inputSize = size(varargin{1},1);
            [varargout{1:nargout}] = Kin2_mex('mapDepthFrame2Color', this.objectHandle, varargin{1},uint32(inputSize));
        end
                
        function varargout = mapDepthPoints2Camera(this, varargin)
            % mapDepthPoints2Camera - map the input points from depth 
            % coordinates to camera space coordinates
            % Input: n x 2 matrix (n points, x,y)
            % Output: n x 3 matrix (n points, x,y,z)
            % See mapping2CamDemo.m
            inputSize = size(varargin{1},1);
            [varargout{1:nargout}] = Kin2_mex('mapDepthPoints2Camera', this.objectHandle, varargin{1},uint32(inputSize));
        end
        
        
        %% Color mappings
        function varargout = mapColorPoints2Depth(this, varargin)
            % mapColorPoints2Depth - map the input points from color 
            % coordinates to depth coordinates
            % Input and output: n x 2 matrix (n points)
            % See mappingDemo.m
            inputSize = size(varargin{1},1);
            [varargout{1:nargout}] = Kin2_mex('mapColorPoints2Depth', this.objectHandle, varargin{1},uint32(inputSize));
        end
                
        function varargout = mapColorPoints2Camera(this, varargin)
            % mapColorPoints2Camera - map the input points from color 
            % coordinates to camera space coordinates
            % Input: n x 2 matrix (n points, x,y)
            % Output: n x 3 matrix (n points, x,y,z)
            % See mapping2CamDemo.m
            inputSize = size(varargin{1},1);
            [varargout{1:nargout}] = Kin2_mex('mapColorPoints2Camera', this.objectHandle, varargin{1},uint32(inputSize));
        end
        
        %% Camera mappings         
        function varargout = mapCameraPoints2Depth(this, varargin)
            % mapCameraPoints2Depth - map the input points from camera space 
            % coordinates to depth coordinates
            % Input: n x 3 matrix (n points, x,y,z)
            % Output: n x 2 matrix (n points, x,y)
            % See mapping2CamDemo.m
            inputSize = size(varargin{1},1);
            [varargout{1:nargout}] = Kin2_mex('mapCameraPoints2Depth', this.objectHandle, varargin{1},uint32(inputSize));
        end
        
        function varargout = mapCameraPoints2Color(this, varargin)
            % mapCameraPoints2Color - map the input points from camera space 
            % coordinates to color coordinates
            % Input: n x 3 matrix (n points, x,y,z)
            % Output: n x 2 matrix (n points, x,y)
            % See mapping2CamDemo.m
            inputSize = size(varargin{1},1);
            [varargout{1:nargout}] = Kin2_mex('mapCameraPoints2Color', this.objectHandle, varargin{1},uint32(inputSize));
        end
        
        %% Skeleton functions
                
        function varargout = getBodies(this, varargin)
            % getBodies - Get 3D bodies joints 
            % Input: none
            % Output: Structure array.
            % The structure array (bodies) contains 6 bodies at most
            % Each body has:
            % -Position: 3x25 matrix containing the x,y,z of the 25 joints in
            %   camera space coordinates
            % -TrackingState: state of each joint. These can be:
            %   NotTracked=0, Inferred=1, or Tracked=2
            % -LeftHandState: state of the left hand
            % -RightHandState: state of the right hand
            % See bodyDemo.m
            [varargout{1:nargout}] = Kin2_mex('getBodies', this.objectHandle, varargin{:});
        end
        
        function drawBodies(this,handle,bodies,destination,jointsSize, ...
                bonesThickness,handsSize)
            % drawBodies - % Draw bodies on depth image
            % Input Parameters: 
            % 1) handle: image axes handle
            % 2) bodies: bodies structure returned by getBodies method
            % 3) destination: destination image (depth or color)
            % 4) jointsSize: joints' size (circle raddii)
            % 5) bonesThickness: Bones' Thickness
            % 6) handsSize: Hands' Size
            % Output: none
            % See bodyDemo.m
            numBodies = size(bodies,2);
            
            % Draw each body
            for i=1:numBodies                
                if strcmp(destination,'depth')
                    % Get the joints in depth image space
                    pos2D = this.mapCameraPoints2Depth(bodies(i).Position');
                elseif strcmp(destination,'color')
                    pos2D = this.mapCameraPoints2Color(bodies(i).Position');
                end
                
                % Get the bones: (x1,x2)(y1,y2) of each bone to be drawns
                % with lines
                [bonesx, bonesy] = this.getBones(pos2D);                                    

                % Draw the joints
                viscircles(handle,pos2D,ones(25,1)*jointsSize,'EdgeColor',this.bodyColors(i));
                %plot(handle,pos2D(:,1),pos2D(:,2), ...
                %'LineStyle','none','Marker','o','MarkerFaceColor',this.bodyColors(i), ...
                %'MarkerEdgeColor',this.bodyColors(i),'MarkerSize', jointsSize);

                % Draw the bones
                for j=1:24                    
                    line(bonesx(:,j),bonesy(:,j),'Color',this.bodyColors(i), ...
                        'LineWidth',bonesThickness,'Parent',handle);
                end

                % Draw the hands
                this.drawHand(handle, bodies(i).LeftHandState, pos2D(this.JointType_HandLeft,:),handsSize);
                this.drawHand(handle, bodies(i).RightHandState, pos2D(this.JointType_HandRight,:),handsSize);
                                        
            end
        end
                
        function [bonesx, bonesy] = getBones(this,joints)
            % getBones - Get the bones pair of coordinates: (x1,x2)(y1,y2) 
            % to be drawn with lines
            % Input: 2D joints
            % output: 24 (x,y) coordinates. 
            bonesx = zeros(2,24);
            bonesy = zeros(2,24);
            % Torso
            bonesx(:,1) = [joints(this.JointType_Head,1); joints(this.JointType_Neck,1)];
            bonesy(:,1) = [joints(this.JointType_Head,2); joints(this.JointType_Neck,2)];
            bonesx(:,2) = [joints(this.JointType_Neck,1); joints(this.JointType_SpineShoulder,1)];
            bonesy(:,2) = [joints(this.JointType_Neck,2); joints(this.JointType_SpineShoulder,2)];
            bonesx(:,3) = [joints(this.JointType_SpineShoulder,1); joints(this.JointType_SpineMid,1)];
            bonesy(:,3) = [joints(this.JointType_SpineShoulder,2); joints(this.JointType_SpineMid,2)];
            bonesx(:,4) = [joints(this.JointType_SpineMid,1); joints(this.JointType_SpineBase,1)];
            bonesy(:,4) = [joints(this.JointType_SpineMid,2); joints(this.JointType_SpineBase,2)];
            bonesx(:,5) = [joints(this.JointType_SpineShoulder,1); joints(this.JointType_ShoulderRight,1)];
            bonesy(:,5) = [joints(this.JointType_SpineShoulder,2); joints(this.JointType_ShoulderRight,2)];
            bonesx(:,6) = [joints(this.JointType_SpineShoulder,1); joints(this.JointType_ShoulderLeft,1)];
            bonesy(:,6) = [joints(this.JointType_SpineShoulder,2); joints(this.JointType_ShoulderLeft,2)];
            bonesx(:,7) = [joints(this.JointType_SpineBase,1); joints(this.JointType_HipRight,1)];
            bonesy(:,7) = [joints(this.JointType_SpineBase,2); joints(this.JointType_HipRight,2)];
            bonesx(:,8) = [joints(this.JointType_SpineBase,1); joints(this.JointType_HipLeft,1)];
            bonesy(:,8) = [joints(this.JointType_SpineBase,2); joints(this.JointType_HipLeft,2)];
            
            % Right Arm
            bonesx(:,9) = [joints(this.JointType_ShoulderRight,1); joints(this.JointType_ElbowRight,1)];
            bonesy(:,9) = [joints(this.JointType_ShoulderRight,2); joints(this.JointType_ElbowRight,2)];
            bonesx(:,10) = [joints(this.JointType_ElbowRight,1); joints(this.JointType_WristRight,1)];
            bonesy(:,10) = [joints(this.JointType_ElbowRight,2); joints(this.JointType_WristRight,2)];
            bonesx(:,11) = [joints(this.JointType_WristRight,1); joints(this.JointType_HandRight,1)];
            bonesy(:,11) = [joints(this.JointType_WristRight,2); joints(this.JointType_HandRight,2)];
            bonesx(:,12) = [joints(this.JointType_HandRight,1); joints(this.JointType_HandTipRight,1)];
            bonesy(:,12) = [joints(this.JointType_HandRight,2); joints(this.JointType_HandTipRight,2)];
            bonesx(:,13) = [joints(this.JointType_WristRight,1); joints(this.JointType_ThumbRight,1)];
            bonesy(:,13) = [joints(this.JointType_WristRight,2); joints(this.JointType_ThumbRight,2)];
                                                         
            % Left Arm
            bonesx(:,14) = [joints(this.JointType_ShoulderLeft,1); joints(this.JointType_ElbowLeft,1)];
            bonesy(:,14) = [joints(this.JointType_ShoulderLeft,2); joints(this.JointType_ElbowLeft,2)];
            bonesx(:,15) = [joints(this.JointType_ElbowLeft,1); joints(this.JointType_WristLeft,1)];
            bonesy(:,15) = [joints(this.JointType_ElbowLeft,2); joints(this.JointType_WristLeft,2)];
            bonesx(:,16) = [joints(this.JointType_WristLeft,1); joints(this.JointType_HandLeft,1)];
            bonesy(:,16) = [joints(this.JointType_WristLeft,2); joints(this.JointType_HandLeft,2)];
            bonesx(:,17) = [joints(this.JointType_HandLeft,1); joints(this.JointType_HandTipLeft,1)];
            bonesy(:,17) = [joints(this.JointType_HandLeft,2); joints(this.JointType_HandTipLeft,2)];
            bonesx(:,18) = [joints(this.JointType_WristLeft,1); joints(this.JointType_ThumbLeft,1)];
            bonesy(:,18) = [joints(this.JointType_WristLeft,2); joints(this.JointType_ThumbLeft,2)];                                            
            
            % Right Leg
            bonesx(:,19) = [joints(this.JointType_HipRight,1); joints(this.JointType_KneeRight,1)];
            bonesy(:,19) = [joints(this.JointType_HipRight,2); joints(this.JointType_KneeRight,2)];
            bonesx(:,20) = [joints(this.JointType_KneeRight,1); joints(this.JointType_AnkleRight,1)];
            bonesy(:,20) = [joints(this.JointType_KneeRight,2); joints(this.JointType_AnkleRight,2)];
            bonesx(:,21) = [joints(this.JointType_AnkleRight,1); joints(this.JointType_FootRight,1)];
            bonesy(:,21) = [joints(this.JointType_AnkleRight,2); joints(this.JointType_FootRight,2)];                        
            
            % Left Leg
            bonesx(:,22) = [joints(this.JointType_HipLeft,1); joints(this.JointType_KneeLeft,1)];
            bonesy(:,22) = [joints(this.JointType_HipLeft,2); joints(this.JointType_KneeLeft,2)];
            bonesx(:,23) = [joints(this.JointType_KneeLeft,1); joints(this.JointType_AnkleLeft,1)];
            bonesy(:,23) = [joints(this.JointType_KneeLeft,2); joints(this.JointType_AnkleLeft,2)];
            bonesx(:,24) = [joints(this.JointType_AnkleLeft,1); joints(this.JointType_FootLeft,1)];
            bonesy(:,24) = [joints(this.JointType_AnkleLeft,2); joints(this.JointType_FootLeft,2)];                                    
            
        end
                
        function drawHand(this,handle, handState, handPos,size)
            % drawHand - draw the hand
            % Input parameters:
            % 1) handle: image axes handle
            % 2) handState: hand state, obtained from bodies structure (RightHandState
            % or LeftHandState elements of the bodies structure)
            % 3) handPos: hand position, obtain from the bodies structure
            % 4) size: Circle radii
            color = [];
            if handState == this.HandState_Closed         
                color = 'r';
            elseif handState == this.HandState_Open
                color = 'g';
            elseif handState == this.HandState_Lasso                
                color = 'b';
            end
            
            if ~isempty(color)
                viscircles(handle,handPos,ones(1,1)*size,'EdgeColor',color);
                %plot(handle,handPos(1),handPos(2), ...
                %    'LineStyle','none','Marker','o','MarkerFaceColor',color, ...
                 %   'MarkerEdgeColor',color,'MarkerSize', 15);
            end
        end
        
        %% Face Processing
        function drawFaces(this,handle,faces,pointsSize,displayText,fontSize)
            % drawFaces: Display the faces data
            % Input parameters: 
            % 1) handle: image axes
            % 2) faces: faces structure obtained with getFaces
            % 3) pointsSize: face landmarks size (radius)
            % 4) displayText: display text information?
            % 5) fontSize: text font size in pixels
            % See faceDemo.m
            disp(nargin)
            if nargin < 6
                fontSize = 20;
            end

            numFaces = size(faces,2);
                        
            % Draw each body
            for i=1:numFaces                
                % Draw the facial landmarks
                viscircles(handle,faces(i).FacePoints',ones(5,1)*pointsSize,'EdgeColor',this.bodyColors(i));

                % Draw the rectangle    
                left = faces(i).FaceBox(1);
                top = faces(i).FaceBox(2);
                right = faces(i).FaceBox(3);
                bottom = faces(i).FaceBox(4);
                  % top line
                line([left right],[top top],'Color',this.bodyColors(i), ...
                    'LineWidth',2,'Parent',handle);
                  % bottom line
                line([left right],[bottom bottom],'Color',this.bodyColors(i), ...
                    'LineWidth',2,'Parent',handle);
                  % left line
                line([left left],[top bottom],'Color',this.bodyColors(i), ...
                    'LineWidth',2,'Parent',handle);
                  % right line
                line([right right],[top bottom],'Color',this.bodyColors(i), ...
                    'LineWidth',2,'Parent',handle);
                
                if displayText
                    % Display face information
                    for j=1:length(this.faceProperties)
                        yoffset = j*fontSize*1.5;
                        p = faces(i).FaceProperties(j);
                        strp = this.decodeFaceProperties(p);
                        str = strcat(this.faceProperties(j),':',strp);
                        text(left,bottom + yoffset ,str, ...
                          'Color',this.bodyColors(i),'FontSize',fontSize, ...
                          'FontUnits','pixels','Parent',handle);
                    end

                    % Display face rotation
                    pitch = faces(i).FaceRotation(1);
                    yaw = faces(i).FaceRotation(2);
                    roll = faces(i).FaceRotation(3);
                    str = ['Pitch:' num2str(pitch) ' Yaw:' num2str(yaw) ...
                        ' Roll:' num2str(roll)];
                    text(left,bottom + yoffset + fontSize*1.5 ,str, ...
                     'Color',this.bodyColors(i),'FontSize',fontSize, ...
                     'FontUnits','pixels','Parent',handle);
                end
            end
        end
        
        function str = decodeFaceProperties(this,detectionResult)
            % decodeFaceProperties: return description of a numeric face
            % property. This function can be used to display the name of
            % the properties. See method drawFaces for usage example
            switch(detectionResult)
                case this.DetectionResult_Unknown 
                    str = 'Unknown';
                case this.DetectionResult_No 
                    str = 'No';
                case this.DetectionResult_Maybe 
                    str = 'Maybe';
                case this.DetectionResult_Yes 
                    str = 'Yes';
                otherwise
                    str = 'Unknown';
            end
        end
        
        function drawHDFaces(this,handle,faces,displayPoints,displayText,fontSize)
            % drawHDFaces: display the HD faces data and face model
            % Input Parameters: 
            % 1) handle: image axes
            % 2) faces: structure obtained with getFaces
            % 3) displayPoints: display HD face model vertices(1347 points)?
            % 4) displayText: display text information (animation units)?
            % 5) fontSize: text font size in pixels
            % See faceHDDemo.m
            disp(nargin)
            if nargin < 6
                fontSize = 20;
            end

            numFaces = size(faces,2);
                        
            % Draw each body
            for i=1:numFaces                                
                % Draw the rectangle if not showing the points    
                left = faces(i).FaceBox(1);
                top = faces(i).FaceBox(2);
                right = faces(i).FaceBox(3);
                bottom = faces(i).FaceBox(4);
                
                if ~displayPoints
                  % top line
                    line([left right],[top top],'Color',this.bodyColors(i), ...
                        'LineWidth',2,'Parent',handle);
                      % bottom line
                    line([left right],[bottom bottom],'Color',this.bodyColors(i), ...
                        'LineWidth',2,'Parent',handle);
                      % left line
                    line([left left],[top bottom],'Color',this.bodyColors(i), ...
                        'LineWidth',2,'Parent',handle);
                      % right line
                    line([right right],[top bottom],'Color',this.bodyColors(i), ...
                        'LineWidth',2,'Parent',handle);
                end
                
                if displayText
                    % Display face information
                    for j=1:length(this.faceAnimationUnits)
                        yoffset = j*fontSize*1.5;
                        p = faces(i).AnimationUnits(j);
                        str = strcat(this.faceAnimationUnits(j),':',num2str(p));
                        text(right,top + yoffset ,str, ...
                          'Color',this.bodyColors(i),'FontSize',fontSize, ...
                          'FontUnits','pixels','Parent',handle);
                    end

                    % Display face rotation
                    pitch = faces(i).FaceRotation(1);
                    yaw = faces(i).FaceRotation(2);
                    roll = faces(i).FaceRotation(3);
                    str = ['Pitch:' num2str(pitch) ' Yaw:' num2str(yaw) ...
                        ' Roll:' num2str(roll)];
                    yoffset = yoffset + fontSize*1.5;
                    text(right,top + yoffset ,str, ...
                     'Color',this.bodyColors(i),'FontSize',fontSize, ...
                     'FontUnits','pixels','Parent',handle);
                 
                 % Display head pivot
                    x = faces(i).HeadPivot(1);
                    y = faces(i).HeadPivot(2);
                    z = faces(i).HeadPivot(3);
                    str = ['Head Pivot:' num2str(x) ',' num2str(y) ...
                        ',' num2str(z)];
                    yoffset = yoffset + fontSize*1.5;
                    text(right,top + yoffset ,str, ...
                     'Color',this.bodyColors(i),'FontSize',fontSize, ...
                     'FontUnits','pixels','Parent',handle);
                end
                
                % Display HD points on color image
                if displayPoints
                    colorCoords = this.mapCameraPoints2Color(faces(i).FaceModel');
                    viscircles(handle,colorCoords,ones(1347,1)*1,'EdgeColor',this.bodyColors(i));
                end

            end
        end
        
        %% Kinect Fusion
         function varargout = KF_init(this, varargin)
            % KF_init - Initialize Kinect Fusion
            % See kinectFusionDemo.m
            [varargout{1:nargout}] = Kin2_mex('KF_init', this.objectHandle, varargin{:});
         end
        
         function varargout = KF_update(this, varargin)
            % KF_update - Update reconstruction
            % See kinectFusionDemo.m
            [varargout{1:nargout}] = Kin2_mex('KF_update', this.objectHandle, varargin{:});
         end
         
         function varargout = KF_getVolumeImage(this, varargin)
            % KF_getVolumeImage - Obtains a raycast image
            % See kinectFusionDemo.m
            [varargout{1:nargout}] = Kin2_mex('KF_getVolumeImage', this.objectHandle, varargin{:});
         end
         
         function varargout = KF_getMesh(this, varargin)
            % KF_getMesh - Obtains a mesh of the Kinectu Fusion volume
            % See kinectFusionDemo.m
            [varargout{1:nargout}] = Kin2_mex('KF_getMesh', this.objectHandle, varargin{:});
         end
         
         function varargout = KF_reset(this, varargin)
            % KF_reset - Reset volume and pose
            % See kinectFusionDemo.m
            [varargout{1:nargout}] = Kin2_mex('KF_reset', this.objectHandle, varargin{:});
         end
    end
end