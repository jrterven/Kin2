classdef Kin2Collector < Kin2
    % Kin2Collector  Subclass of Kin2 that adds data capture functionality
    %   Use this class to collect data from your Kinect with the aid of
    %   timer schedulers that manage calls into Kin2.updateData() and
    %   Kin2.get<data>() methods.
    %
    % Example (streaming):
    %   k2 = Kin2Collector('body');
    %   k2.startStreaming();
    %   % k2.<data> and k2.<data>_log propeties update without blocking
    %   k2.stopStreaming();
    %   k2.delete();
    %   clear k2;
    %
    % Example (callback):
    %   k2 = Kin2Collector('body');
    %   k2.newDataCallback = @(src,evt) fprintf('log size = % 6d\n',src.count);
    %   k2.startStreaming();
    %   % k2.<data> and k2.<data>_log propeties update without blocking
    %   % you'll see newDataCallback print the count in increments of 1
    %   % as new data is received
    %   k2.stopStreaming();
    %   k2.delete();
    %   clear k2;
    
    properties (SetAccess=public)
        
        % newDataCallback  Called when new data received
        %   Assign a function handle with the signature func(src,evt) or
        %   the empty array to remove the callback.
        newDataCallback = [];
        
    end
    
    properties (SetAccess=private)
        
        % the so-called '<data>' properties
        time        = [];
        depth       = [];
        color       = [];
        bodies      = [];
        bodyIndex   = [];
        bones       = [];
        faces       = [];
        hdFaces     = [];
        infrared    = [];
        pointCloud  = []; % TODO: implement this
        
    end
    
    properties (Dependent)
        
        % timeNow  Time since class instantiation in seconds
        %   Based on the built-in command 'now'
        timeNow;
        
        % timeNowStreaming  Time since startStreaming() in seconds
        %   Based on the built-in command 'now'
        timeNowStreaming;
        
        % rateAverage  Average streaming rate in Hz (or fps)
        %   Based on built-in command 'now' and the time since the most
        %   recent call to the startStreaming() method.
        rateAverage;
        
    end
    
    properties (SetAccess=private,Hidden=true)
        
        % timeInit  Time when object is instantiated in seconds
        %   Based on the built-in command 'now'
        %   This is used to compute the dependent property timeNow
        timeInit;
        
        % timeInitStreaming  Time when startStreaming() was called in
        % seconds
        timeInitStreaming;
        
        % the so-called '<data>_log' properties
        time_log        = {};
        depth_log       = {};
        color_log       = {};
        bodies_log      = {};
        bodyIndex_log   = {};
        bones_log       = {};
        faces_log       = {};
        hdFaces_log     = {};
        infrared_log    = {};
        pointCloud_log  = {};
        
        streamingCount = 0;
        
        streamingTimer = [];
        
    end
    
    methods
        
        %% --- Object Management
        function this = Kin2Collector(varargin)
            this = this@Kin2(varargin{:});
            this.timeInit = now*24*60*60;
        end
        
        function delete(this)
            % stop streaming and clean up timer
            if ~isempty(this.streamingTimer)
                stop(this.streamingTimer);
                delete(this.streamingTimer);
                this.streamingTimer = [];
            end
            % Note (for my future self):
            %   The superclass (Kin2) delete method is called by MATLAB
            %   automatically. This method does not override Kin2.delete(),
            %   so all we do here is clean up Kin2Collector stuff. See:
            %   http://www.mathworks.com/help/matlab/matlab_oop/handle-class-destructors.html
            % mark-toma
        end
        
        function clearLogs(this)
            
            % Let's let the logs clear while streaming
            %if ~isempty(this.streamingTimer)
            %    error('Cannot clear logs while streaming.');
            %end
            this.time_log        = {};
            this.depth_log       = {};
            this.color_log       = {};
            this.bodies_log      = {};
            this.bodyIndex_log   = {};
            this.bones_log       = {};
            this.faces_log       = {};
            this.hdFaces_log     = {};
            this.infrared_log    = {};
            this.pointCloud_log  = {};
        end
        
        %% --- Getters
        function val = get.timeNow(this)
            val = now*24*60*60-this.timeInit;
        end
        
        function val = get.timeNowStreaming(this)
            val = this.timeNow - this.timeInitStreaming;
        end
        
        function val = get.rateAverage(this)
            if isempty(this.streamingTimer), val = []; return; end
            val = this.streamingCount/(this.timeNow - this.timeInitStreaming);
        end
        
        %% --- Setters
        function set.newDataCallback(this,func)
            if isempty(func)
                % unset callback
                this.newDataCallback = [];
            elseif isa(func,'function_handle') && (2 == nargin(func))
                % set callback
                this.newDataCallback = func;
            else
                % we got a problem here...
                emsg = [...
                    'Property newDataCallback must be set to empty (to remove callback) ',...
                    'or a function handle accepting two input arguments (to set callback) ',...
                    'conforming to the signature newDataCallback(src,evt).'];
                error(emsg);
            end
        end
        
        function set.time_log(this,val)
            if isempty(this.time_log)
                this.time_log = {val};
            else
                this.time_log = [this.time_log;{val}];
            end
        end
        
        function set.depth_log(this,val)
            if isempty(this.depth_log)
                this.depth_log = {val};
            else
                this.depth_log = [this.depth_log;{val}];
            end
        end
        
        function set.color_log(this,val)
            if isempty(this.color_log)
                this.color_log = {val};
            else
                this.color_log = [this.color_log;{val}];
            end
        end
        
        function set.bodies_log(this,val)
            if isempty(this.bodies_log)
                this.bodies_log = {val};
            else
                this.bodies_log = [this.bodies_log;{val}];
            end
        end
        
        function set.bodyIndex_log(this,val)
            if isempty(this.bodyIndex_log)
                this.bodyIndex_log = {val};
            else
                this.bodyIndex_log = [this.bodyIndex_log;{val}];
            end
        end
        
        function set.bones_log(this,val)
            if isempty(this.bones_log)
                this.bones_log = {val};
            else
                this.bones_log = [this.bones_log;{val}];
            end
        end
        
        function set.faces(this,val)
            if isempty(this.faces_log)
                this.faces_log = {val};
            else
                this.faces_log = [this.faces_log;{val}];
            end
            this.faces = val;
        end
        
        function set.hdFaces_log(this,val)
            if isempty(this.hdFaces_log)
                this.hdFaces_log = {val};
            else
                this.hdFaces_log = [this.hdFaces_log;{val}];
            end
        end
        
        function set.infrared_log(this,val)
            if isempty(this.infrared_log)
                this.infrared_log = {val};
            else
                this.infrared_log = [this.infrared_log;{val}];
            end
        end
        
        function set.pointCloud_log(this,val)
            if isempty(this.pointCloud_log)
                this.pointCloud_log = {val};
            else
                this.pointCloud_log = [this.pointCloud_log;{val}];
            end
        end
        
        %% --- Data Streaming / Polling
        function startStreaming(this)
            
            if ~isempty(this.streamingTimer)
                error('Cannot start streaming when already streaming.');
            end
            
            this.streamingTimer = timer(...
                'busymode','queue',...
                'timerfcn',@this.streamingCallback,...
                'executionmode','fixedrate',...
                'name','Kin2_Collector_Streaming_Timer',...
                'period',0.01,...
                'startdelay',1);
            
            % init state to log info on streaming data
            this.streamingCount = 0;
            
            start(this.streamingTimer);
            
        end
        
        function stopStreaming(this)
            % stopStreaming
            if isempty(this.streamingTimer)
                error('Cannot stop streaming when not streaming.');
            end
            
            stop(this.streamingTimer);
            delete(this.streamingTimer);
            this.streamingTimer = [];
            
            this.timeInitStreaming = [];
            
        end
        
        function pollData(this)
            % pollData  Blocks while waiting for 
            tic;
            while ~this.updateData
                if toc>5 % error out if timeout occurs
                    error('Polling for data took excessively long time... something''s wrong');
                end
            end
            this.fetchData();
        end
        
    end
    
    methods (Access=private)
        
        function fetchData(this)
            
            this.time = this.timeNow;
            this.time_log = this.time;
                        
            % update with Kin2_mex data if source is selected
            % TODO:
            %   bones and point cloud need flags or something
            if this.flag_depth
                this.depth = this.getDepth();
                this.depth_log = this.depth;
            end
            if this.flag_color
                this.color = this.getColor(); 
                this.color_log = this.color;
            end
            if this.flag_body
                this.bodies = this.getBodies(); 
                this.bodies_log = this.bodies;
            end
            if this.flag_bodyIndex
                this.bodyIndex = this.getBodyIndex(); 
                this.bodyIndex_log = this.bodyIndex;
            end
            %if this.flag_bones
            %    this.bones = this.getBones();
            %    this.bones_log = this.bones;
            %end
            if this.flag_face
                this.faces = this.getFaces(); 
                this.faces_log = this.faces;
            end
            if this.flag_hd_face
                this.hdFaces = this.getHDFaces(); 
                this.hdFaces_log = this.hdFaces;
            end
            if this.flag_infrared
                this.infrared = this.getInfrared(); 
                this.infrared_log = this.infrared;
            end
            %if this.flag_point_cloud
            %    this.pointCloud = this.getPointCloud();
            %    this.pointCloud_log = this.pointCloud;
            %end
            
        end        
        
        function streamingCallback(this,~,~)
            
            % bail if no new data
            if ~this.updateData(), return; end
            
            this.fetchData(); % get data from Kin2_mex
            
            this.streamingCount = this.streamingCount + 1;
            if isempty(this.timeInitStreaming)
                this.timeInitStreaming = this.time;
            end
            
            % callback
            if ~isempty(this.newDataCallback)
                this.newDataCallback(this,[]);
            end
            
        end
        
    end    
    
end