% Kin2 Class
% Kin2 provides access to Kinect V2 functionality using the Kinect For
% Windows SDK 2.0
%
% See the demos for its usage.
% See README.txt file for a brief description of each demo.
% 
% Authors:
% Juan R. Terven, jrterven@hotmail.com
% Diana M. Cordova, diana_mce@hotmail.com
% 
% Citation:
% J. R. Terven, D. M. Cordova, "A Kinect 2 Toolbox for MATLAB", 
% https://github.com/jrterven/Kin2, 2016.
% 
classdef Kin2 < handle
    %KIN2 MATLAB class wrapper to an underlying C++ Kin2 class
    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
        
        % Bodies colors
        bodyColors = ['r','b','g','y','m','c'];
        
        % Selected sources
        flag_depth = false;
        flag_color = false;
        flag_infrared = false;
        flag_bodyIndex = false;
        flag_body = false;
        flag_face = false;
        flag_hd_face = false;
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
        
        % Face Builder Collection Status
        FaceModelBuilderCollectionStatus_Complete               = 0,
        FaceModelBuilderCollectionStatus_MoreFramesNeeded       = 1,
        FaceModelBuilderCollectionStatus_FrontViewFramesNeeded	= 2,
        FaceModelBuilderCollectionStatus_LeftViewsNeeded        = 4,
        FaceModelBuilderCollectionStatus_RightViewsNeeded       = 8,
        FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded	= 16
        
        % Face Builder Capture Status
        FaceModelBuilderCaptureStatus_GoodFrameCapture	= 0,
        FaceModelBuilderCaptureStatus_OtherViewsNeeded	= 1,
        FaceModelBuilderCaptureStatus_LostFaceTrack     = 2,
        FaceModelBuilderCaptureStatus_FaceTooFar        = 3,
        FaceModelBuilderCaptureStatus_FaceTooNear       = 4,
        FaceModelBuilderCaptureStatus_MovingTooFast     = 5,
        FaceModelBuilderCaptureStatus_SystemError       = 6        
        
        % Color calibration initialization parameters
        calibParams = struct;
        colorCalib = false;     % Is color calibration already calculated?
        colorFL = 1000;         % focal length
        colorPPX = 960;         % ppx
        colorPPY = 540;         % ppy  
        colorRot = eye(3);      % rotation of color camera wrt depth camera
        colorTranslation = [0 0 0]; % translation of color camera wrt depth
        colork1 = 0; colork2 = 0; colork3 = 0; % radial distortion parameters
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
            this.flag_depth = ismember('depth',varargin);
            this.flag_color = ismember('color',varargin);
            this.flag_infrared = ismember('infrared',varargin);
            this.flag_bodyIndex = ismember('body_index',varargin);
            this.flag_body = ismember('body',varargin);
            this.flag_face = ismember('face',varargin);
            this.flag_hd_face = ismember('HDface',varargin);
            flags = uint16(0);
            
            % Extract flags values (from Kinect.h)
%             FrameSourceTypes_None	= 0,
%             FrameSourceTypes_Color	= 0x1,
%             FrameSourceTypes_Infrared	= 0x2,
%             FrameSourceTypes_Depth	= 0x8,
%             FrameSourceTypes_BodyIndex	= 0x10,
%             FrameSourceTypes_Body	= 0x20,
            if this.flag_color, flags = flags + 1; end
            if this.flag_infrared, flags = flags + 2; end
            if this.flag_depth, flags = flags + 8; end
            if this.flag_bodyIndex, flags = flags + 16; end
            if this.flag_body, flags = flags + 32; end    
            if this.flag_face, flags = flags + 128; end
            if this.flag_hd_face, flags = flags + 256; end
            
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
            % updateData before and verify that there is valid data.
            % See videoDemo.m
            [varargout{1:nargout}] = Kin2_mex('getDepth', this.objectHandle, varargin{:});
        end
                
        function varargout = getColor(this, varargin)
            % getColor - return color frame from Kinect2. You must call
            % updateData before and verify that there is valid data.
            % See videoDemo.m
            [varargout{1:nargout}] = Kin2_mex('getColor', this.objectHandle, varargin{:});
        end
                
        function varargout = getInfrared(this, varargin)
            % getInfrared - return infrared frame from Kinect2. You must call
            % updateData before and verify that there is valid data.
            % See videoDemo.m
            [varargout{1:nargout}] = Kin2_mex('getInfrared', this.objectHandle, varargin{:});
        end
        
        %% Data Sources
        function varargout = getBodyIndex(this, varargin)
            % getBodyIndex - return body index frame from Kinect2. You must call
            % updateData before and verify that there is valid data.
            % See bodyIndexDemo.m
            [varargout{1:nargout}] = Kin2_mex('getBodyIndex', this.objectHandle, varargin{:});
        end
        
        function varargout = getPointCloud(this, varargin)
            % getPointCloud - return an 217088nx3 point cloud or a MATLAB 
            % built-in pointCloud object.
            % Name-Value Pair Arguments: 
            %   'output' - output format of the pointcloud
            %       'raw'(default) | 'pointCloud'
            %   The 'raw' output consists of an 217088nx3 points.
            %   The 'pointCloud' output consist of a pointCloud object.
            %   Note that this object was introduced with MATLAB 2015b.
            %   Earlier versions will not support this type of output.
            %
            %   'color' - boolean value indicating if we want the colors of
            %   each point of the point cloud. 
            %       'false'(default) | 'true'
            %   If 'color' is true and 'output' is 'raw', this method
            %   returns two separate 217088nx3 matrices. One with the x,y,z
            %   values of each point and the other with the R,G,B values of
            %   each point.
            %   If 'color' is true and 'output' is 'pointCloud', this
            %   method return a pointCloud object with the color embedded.
            %   Note that if 'color' is true, you must activate the color
            %   camera on the Kin2 object creation. Otherwise it will
            %   trigger a warning each time the method is called.
            %
            %   You must call updateData before and verify that there is valid data.
            %   See pointCloudDemo.m and pointCloudDemo2.m
            
            % Parse inputs
            p = inputParser;
            defaultOutput = 'raw';
            expectedOutputs = {'raw','pointCloud'};
            defaultColor = 'false';
            expectedColors = {'true','false'};
            
            addParameter(p,'output',defaultOutput,@(x) any(validatestring(x,expectedOutputs)));
            addParameter(p,'color',defaultColor,@(x) any(validatestring(x,expectedColors)));
            parse(p,varargin{:});
            
            % Required color?
            if strcmp(p.Results.color,'true')
                % If not color source selected, display a warning
                if ~this.flag_color
                    warning(['color source is not selected.' ...
                        ' Please select the color source when creating Kin2 object.']);
                    withColor = uint32(0);
                else
                    withColor = uint32(1);
                end
                
            else
                withColor = uint32(0);
            end
            
            % Get the pointcloud from the Kinect V2 as a nx3 matrix
            [varargout{1:2}] = Kin2_mex('getPointCloud', this.objectHandle, withColor);
            
            % If the required output is a pointCloud object,            
            if strcmp(p.Results.output,'pointCloud')
                % check if this version of MATLAB suppor the pointCloud object
                if exist('pointCloud','class') ~= 8
                    this.delete;
                    error('This version of Matlab do not Support pointCloud object.')
                % pointCloud object supported!
                else
                    % Convert nx3 matrix to pointCloud MATLAB object with colors
                    if withColor == 1
                        varargout{1} = pointCloud(varargout{1},'Color',varargout{2});
                    else
                        varargout{1} = pointCloud(varargout{1});
                    end
                end  
            end                     
        end
        
        function varargout = getFaces(this, varargin)
            % getFaces - return structure array with the properties for
            % each found face. You must call updateData before and verify that
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
            % You must call updateData before and verify that there is valid data
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
            % - FaceModel (optional): 3 x 1347 points of a 3D face model computed by face capture
            %   Use 'WithVertices','true' or 'false' to get the face model
            %   or ignore the face model.
            % See faceHDDemo.m                    
            
            % Required Vertices?
            p = inputParser;
            defaultOutput = 'true';
            expectedOutputs = {'true','false'};
            
            addParameter(p,'WithVertices',defaultOutput,@(x) any(validatestring(x,expectedOutputs)));
            parse(p,varargin{:});
            
            % Required HD vertices?
            if strcmp(p.Results.WithVertices,'true')
                withVertices = 1;
            else
                withVertices = 0;
            end
            
            [varargout{1:nargout}] = Kin2_mex('getHDFaces', this.objectHandle, withVertices);
        end
        
        function modelReady = buildHDFaceModels(this, varargin)  
            % buildHDFaceModels - Face captura capabilities of Kinect SDK 2.0
            % Quoting High definition face tracking: 
            % The API will tell a game developer where the person needs to be 
            % in the camera view, the quality of the frames it captures, 
            % and when it has enough valid frames will calculate the shape 
            % of the user’s face and provide a game developer with those shape values. 
            % The game developer can then use those values to influence 
            % their game character design (e.g. make the player’s in-game character look more like the player).
            %
            % Name-Value Pair Arguments: 
            % 'CollectionStatus' - display model collection status information
            %       'true'(default) | 'false'
            % 'CaptureStatus' - display model capture status information.
            %       'false'(default) | 'true'
            % See faceHDDemo2.m
            
            % Parse inputs
            p = inputParser;
            defaultCollectionStatus = 'true';
            expectedCollectionStatus = {'true','false'};
            defaultCaptureStatus = 'false';
            expectedCaptureStatus = {'true','false'};

            addParameter(p,'CollectionStatus',defaultCollectionStatus,@(x) any(validatestring(x,expectedCollectionStatus)));
            addParameter(p,'CaptureStatus',defaultCaptureStatus,@(x) any(validatestring(x,expectedCaptureStatus)));
            
            parse(p,varargin{:});
                       
            % Call the C++ buildHDFaceModels
            % This function returns two codes:
            % A face model builder collection status (collStatus)
            % A face model builder capture status (captureStatus)
            [collStatus, captureStatus] = Kin2_mex('buildHDFaceModels', this.objectHandle, varargin{:});
            
             if strcmp(p.Results.CollectionStatus,'true')
                 str = this.CollectionStatusCode2Str(collStatus);
                 disp(str);
             end
             
             if strcmp(p.Results.CaptureStatus,'true')
                 str = this.CaptureStatusCode2Str(captureStatus);
                disp(str);
             end
             
              if collStatus == this.FaceModelBuilderCollectionStatus_Complete
                  modelReady = true;
              else
                  modelReady = false;
              end
            
        end                
        
        function varargout = getDepthIntrinsics(this, varargin)
            % getDepthIntrinsics - return the depth camera intrinsic
            % parameters inside a structure containing:
            % FocalLengthX, FocalLengthY, PrincipalPointX, PrincipalPointY,
            % RadialDistortionSecondOrder, RadialDistortionFourthOrder, 
            % RadialDistortionSixthOrder
            %
            % Usage: You must call updateData before and verify that there is valid data.
            % See calibrationDemo.m
            [varargout{1:nargout}] = Kin2_mex('getDepthIntrinsics', this.objectHandle, varargin{:});
        end
        
        function calibParams = getColorIntrinsics(this, varargin)
            % getColorIntrinsics - return the color camera intrinsic and
            % extrinsic parameters inside a structure containing:
            % FocalLengthX, FocalLengthY, PrincipalPointX, PrincipalPointY,
            % Rotation (wrt depth camera), Translation(wrt depth camera), 
            % RadialDistortionSecondOrder, RadialDistortionFourthOrder, 
            % RadialDistortionSixthOrder
            %
            % Usage: You must call updateData before and verify that there is valid data.
            % See calibrationDemo.m
            if this.colorCalib
                calibParams = this.calibParams;
            else
                disp('Auto-Calibrating Color Camera ...')
                clear calibCostFun; % clear persistent variables of calibCostFun
                % Get point cloud
                pointcloud = this.getPointCloud;                        
                proj2d = this.mapCameraPoints2Color(pointcloud);
                rot = false;
                if exist('quat2rotm')
                    rot = true;
                end
                % Generates temporary file
                save('calibData.mat','pointcloud','proj2d','rot');

                % The rotation is expresend in quaternions (w,x,y,z)                
                % Minimize cost function: f, ppx, ppy,
                % w,x,y,z,tx,ty,tz,k1,k2,k3
                x0 = [this.colorFL, this.colorPPX, this.colorPPY, 1,0,0,0,0,0,0,0,0,0];
                options = optimset('Algorithm','levenberg-marquardt');
                
                x = fsolve('calibCostFun',x0,options);
                this.colorFL = x(1);
                this.colorPPX = x(2);
                this.colorPPY = x(3);
                % rotation of color camera wrt depth camera
                q = [x(4) x(5) x(6) x(7)];
                % To convert from Quaternion to Rotation matrix we use quat2rotm
                % introduced in MATLAB 2015a. If this function is not
                % present, we return an identity rotation 
                if exist('quat2rotm')                    
                    this.colorRot = quat2rotm(q); 
                else
                    this.colorRot = eye(3);
                end
                
                % translation of color camera wrt depth
                this.colorTranslation = [x(8) x(9) x(10)]; 
                
                % radial distortion parameters
                this.colork1 = x(11); this.colork2 = x(12); this.colork3 = x(13); 
                
                % build the output structure
                this.calibParams = struct('FocalLengthX',this.colorFL, ...
                    'FocalLengthY',this.colorFL,'PrincipalPointX',this.colorPPX, ...
                    'PrincipalPointY',this.colorPPY, 'Rotation', this.colorRot, ...
                    'Translation', this.colorTranslation, ...
                    'RadialDistortionSecondOrder',this.colork1, ...
                    'RadialDistortionFourthOrder', this.colork2, ...
                    'RadialDistortionSixthOrder', this.colork3);
                
                 calibParams = this.calibParams;
                    
                 % color calibration is calculated only once. We use this
                 % flag to keep track of this.
                 this.colorCalib = true;    
                 
                 % Delete temporary files
                 delete('calibData.mat');
            end
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
            % NOT READY YET
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
            % Input: Type of joints orientation output. It can be 'Quat' or
            % 'Euler'
            % Output: Structure array.
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
            % See bodyDemo.m
            quatType = ismember('Quat',varargin);
            eulerType = ismember('Euler',varargin);
            orientType = uint32(0); % Quaternion by default
            if quatType 
                orientType = uint32(0);      
            elseif(eulerType) 
                orientType = uint32(1); 
            else
                disp('getBodies: Orientation type not specified. Using Quaternion by default');
            end
            [varargout{1:nargout}] = Kin2_mex('getBodies', this.objectHandle, orientType);
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
        
        function str = CollectionStatusCode2Str(this,code)
            switch(code)
                case this.FaceModelBuilderCollectionStatus_Complete 
                    str = 'Face model completed!';
                case this.FaceModelBuilderCollectionStatus_MoreFramesNeeded 
                    str = 'More frames needed';
                case this.FaceModelBuilderCollectionStatus_FrontViewFramesNeeded 
                    str = 'Frontal view frames needed';
                case this.FaceModelBuilderCollectionStatus_LeftViewsNeeded 
                    str = 'Left view frames needed';
                case this.FaceModelBuilderCollectionStatus_RightViewsNeeded 
                    str = 'Right view frames needed';
                case this.FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded 
                    str = 'Tilted up view frames needed';    
                otherwise
                    str = '';
            end
        end
        
        function str = CaptureStatusCode2Str(this,code)
            switch(code)
                case this.FaceModelBuilderCaptureStatus_GoodFrameCapture 
                    str = 'Good frame capture';
                case this.FaceModelBuilderCaptureStatus_OtherViewsNeeded 
                    str = 'Other views needed';
                case this.FaceModelBuilderCaptureStatus_LostFaceTrack 
                    str = 'Lost face track';
                case this.FaceModelBuilderCaptureStatus_FaceTooFar 
                    str = 'Face too far';
                case this.FaceModelBuilderCaptureStatus_FaceTooNear 
                    str = 'Face too near';
                case this.FaceModelBuilderCaptureStatus_MovingTooFast 
                    str = 'User moving too fast';    
                case this.FaceModelBuilderCaptureStatus_SystemError 
                    str = 'System Error!';    
                otherwise
                    str = '';
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

    
    