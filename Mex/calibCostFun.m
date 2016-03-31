
function fun = calibCostFun(x0)
            
    persistent X3d x2d quatRot

    height = 1080;
    width = 1920;

    % The first iteration loads the data
    if isempty(X3d)
        X3d = [];
        x2d = [];                    

        % Get the 3D points from the matching results
        % 3D points as a 3 x n matrix
        load('calibData.mat');
        quatRot = rot;
        X3d = pointcloud';

        % 2D points as a 2 x n matrix
        x2d = double(proj2d)';

        % Remove outliers from X3d in both matrices
        x2dValidCols = ~any( isnan( x2d ) | isinf( x2d ) | x2d > width | x2d < 0, 1 );
        x3dValidCols = ~any( isnan( X3d ) | isinf( X3d ) | X3d > 8, 1 );
        validCols = x2dValidCols & x3dValidCols;

        x2d = x2d(:,validCols);
        X3d = X3d(:,validCols);
    end
    
    f = x0(1); % focal length
    cx = x0(2); % principal point x
    cy = x0(3); % principal point y

    % Rotation
    Rq = [x0(4) x0(5) x0(6) x0(7)];

    if quatRot 
        R = quat2rotm(Rq);
    else
        R = eye(3);
    end

    % Translation
    t = [x0(8);x0(9);x0(10)];

    % Build camera matrix
    intrinsic = [f 0  cx;
                 0  f cy;
                 0  0  1];
             
    k1 = x0(11);
    k2 = x0(12);
    k3 = x0(13);

    N = size(X3d,2);
    Xw = X3d;
    xc = x2d;
    xc(2,:) = height - xc(2,:);

    % Apply extrinsic parameters
    proj = R * Xw  + repmat(t,1,size(Xw,2));

    % Apply Intrinsic parameters to get the projection
    proj = intrinsic * proj;
    % Dehomogenization
    proj = proj ./ repmat(proj(3,:),3,1);

    u = proj(1,:);
    v = proj(2,:);
    ud=xc(1,:);
    vd=xc(2,:);   
    
    % Normalized coordinates in the image plane
    un = (u - cx)/f;
    vn = (v - cy)/f;

    % Calculate the Radial Distortion
    r = sqrt(un.^2 + vn.^2);

    compRad(1,:) = 1 + k1*r.^2 + k2*r.^4 + k3*r.^6;
    compRad(2,:) = 1 + k1*r.^2 + k2*r.^4 + k3*r.^6;

    % Undistort the normalized point coordinates in the image plane
    un_undist = un.*compRad(1,:);
    vn_undist = vn.*compRad(2,:);
    
    % Unormalized the points
    u_undist = (un_undist * f) + cx;
    v_undist = (vn_undist * f) + cy;
    
    % Reprojection error 
    fun(1,:)= u_undist - ud;
    fun(2,:)= v_undist - vd;

    err = fun .* fun;
    err = sum(err(:));
    %disp(sqrt(err/N));
end