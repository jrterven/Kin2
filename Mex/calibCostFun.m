function fun = calibCostFun(x0)
            
    persistent X3d x2d

    height = 1080;
    width = 1920;

    % The first iteration loads the data
    if isempty(X3d)
        X3d = [];
        x2d = [];            

        % Get the 3D points from the matching results
        % 3D points as a 3 x n matrix
        load('calibData.mat');
        X3d = pointcloud';

        % 2D poitns as a 2 x n matrix
        x2d = double(proj2d)';

        % Remove outliers from X3d in both matrices
        x2d(:,any(X3d > 8)) = []; % remove columns with values greater than 8 meters
        x2d(:,any(X3d == inf)) = [];
        x2d(:,any(X3d == -inf)) = [];

        X3d(:,any(X3d > 8)) = []; % remove columns with values greater than 8 meters
        X3d(:,any(X3d == inf)) = [];
        X3d(:,any(X3d == -inf)) = [];

        % Now remove outliers from x2d in both matrices
        X3d(:,any(x2d > width)) = []; 
        X3d(:,any(x2d < 0)) = []; % remove columns with negative values
        X3d(:,any(x2d == inf)) = [];
        X3d(:,any(x2d == -inf)) = [];

        x2d(:,any(x2d > width)) = []; 
        x2d(:,any(x2d < 0)) = []; % remove columns with negative values
        x2d(:,any(x2d == inf)) = [];
        x2d(:,any(x2d == -inf)) = [];
    end

    fx = x0(1); % focal length
    cx = x0(2); % principal point x
    cy = x0(3); % principal point y

    % Rotation
    Rx=[1 0 0;0 cos(x0(4)) sin(x0(4));0 -sin(x0(4)) cos(x0(4))];
    Ry=[cos(x0(5)) 0 -sin(x0(5));0 1 0;sin(x0(5)) 0 cos(x0(5))];
    Rz=[cos(x0(6)) sin(x0(6)) 0;-sin(x0(6)) cos(x0(6)) 0;0 0 1];
    R = Rx*Ry*Rz;

    % Translation
    t = [x0(7);x0(8);x0(9)];

    intrinsic = [fx 0  cx;
                 0  fx cy;
                 0  0  1];

    Xw = X3d;
    xc = x2d;
    xc(2,:) = height - xc(2,:);

    % Apply extrinsic parameters
    proj = R * Xw  + repmat(t,1,size(Xw,2));

    % Apply Intrinsic parameters to get the projection
    proj = intrinsic * proj;

    proj = proj ./ repmat(proj(3,:),3,1);

    fun = proj(1:2,:) - xc;

 %   err = fun .* fun;
 %   err = sum(err(:));
    %disp(sqrt(err/N));
end