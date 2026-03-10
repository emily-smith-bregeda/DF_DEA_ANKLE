%% LocalCoordinates

% unchanged from previousfunction

function [R_Matx, origin, x_axis_unit, y_axis_unit, z_axis_unit] = LocalCoordinates(points)
% LocalCoordinates - Computes a local coordinate system from four defining points

    % Define the new origin and points on the axes
    origin = [points(1,:)];  % Coordinates of the new origin
    x_axis_point = [points(4,:)];  % Point on the x-axis
    y_axis_point = [points(2,:)];  % Point on the y-axis
    z_axis_point = [points(3,:)];  % Point on the z-axis
    
    % Calculate translation vector
    t = origin';
    
    % Calculate unit vectors along the new axes
    x_axis_unit = (x_axis_point - origin) / norm(x_axis_point - origin);
    y_axis_unit = (y_axis_point - origin) / norm(y_axis_point - origin);
    z_axis_unit = (z_axis_point - origin) / norm(z_axis_point - origin);
    
    % Form rotation matrix
    R = [x_axis_unit', y_axis_unit', z_axis_unit'];
    
    % Construct 4x4 transformation matrix
    T = eye(4);  % Initialize as identity matrix
    T(1:3, 1:3) = R;  % Replace rotation submatrix with R
    T(1:3, 4) = t;  % Set translation vector
    R_Matx = T;

end

